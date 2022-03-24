// Support for reading ADC counts from at attached load cell via SPI
//
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/misc.h" // timer_read_time
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "spicmds.h" // spidev_transfer
#include "adc_endstop.h" // adc_endstop_report_sample

enum { CHIP_ADS1263, CHIP_ENUM_MAX };

DECL_ENUMERATION("load_cell_sensor_type", "ads1263", CHIP_ADS1263);

enum { TCODE_ERROR = 0xff };
/// SE == Sample Error ??
enum {
    SE_OVERFLOW, SE_SCHEDULE, SE_SPI_TIME, SE_CRC, SE_DUPELICATE
};
// SA == Sample Angle ??
enum {
    SA_PENDING = 1<<0
};

#define MAX_SPI_READ_TIME timer_from_us(50)
#define SAMPLE_WIDTH 5
#define ERROR_WIDTH 2

struct load_cell {
    struct timer timer;
    uint32_t rest_ticks;
    struct spidev_s *spi;
    struct adc_endstop *adc_endstop;
    uint16_t sequence;
    uint8_t flags, chip_type, data_count, time_shift, overflow;
    uint8_t data[SAMPLE_WIDTH * 10];
};

static struct task_wake wake_load_cell;

// Event handler that wakes load_cell_capture_task() periodically
static uint_fast8_t
load_cell_event(struct timer *timer)
{
    struct load_cell *lc = container_of(timer, struct load_cell, timer);
    uint8_t flags = lc->flags;
    if (lc->flags & SA_PENDING)
        lc->overflow++;
    else
        lc->flags = flags | SA_PENDING;
    sched_wake_task(&wake_load_cell);
    lc->timer.waketime += lc->rest_ticks;
    return SF_RESCHEDULE;
}

void
command_config_load_cell(uint32_t *args)
{
    uint8_t chip_type = args[2];
    if (chip_type >= CHIP_ENUM_MAX) {
        shutdown("Invalid load cell sensor chip type");
    }
    struct load_cell *lc = oid_alloc(args[0], command_config_load_cell
                                     , sizeof(*lc));
    lc->timer.func = load_cell_event;
    lc->spi = spidev_oid_lookup(args[1]);
    if (!spidev_have_cs_pin(lc->spi))
        shutdown("load cell sensor requires cs pin");
    lc->chip_type = chip_type;
    uint8_t adc_endstop_oid = args[3];
    // optional endstop to
    if (adc_endstop_oid != 0) {
        struct adc_endstop *ae = adc_endstop_oid_lookup(adc_endstop_oid);
        lc->adc_endstop = ae;
    }
}
DECL_COMMAND(command_config_load_cell,
             "config_load_cell oid=%c spi_oid=%c load_cell_sensor_type=%c"
             " adc_endstop_oid=%c");

// Report local measurement buffer
static void
load_cell_report(struct load_cell *lc, uint8_t oid)
{
    sendf("load_cell_data oid=%c sequence=%hu data=%*s"
          , oid, lc->sequence, lc->data_count, lc->data);
    lc->data_count = 0;
    lc->sequence++;
}

// Send load_cell_data message if buffer is full
static void
flush_if_full(struct load_cell *lc, uint8_t oid)
{
    // Send load_cell_data message if buffer is full
    if ((lc->data_count + SAMPLE_WIDTH - 1) > ARRAY_SIZE(lc->data)) {
        load_cell_report(lc, oid);
    }
}

// Add an entry to the measurement buffer
static void
append_load_cell_measurement(struct load_cell *lc, uint_fast8_t tcode
                            , uint_fast32_t data)
{
    lc->data[lc->data_count] = tcode;
    lc->data[lc->data_count + 1] = data;
    lc->data[lc->data_count + 2] = data >> 8;
    lc->data[lc->data_count + 3] = data >> 16;
    lc->data[lc->data_count + 4] = data >> 24;
    lc->data_count += SAMPLE_WIDTH;
}

// Add an error indicator to the measurement buffer
static void
add_load_cell_error(struct load_cell *lc, uint_fast8_t error_code)
{
    lc->data[lc->data_count] = TCODE_ERROR;
    lc->data[lc->data_count + 1] = error_code;
    lc->data_count += ERROR_WIDTH;
    // endstop is optional, report if enabled
    if (lc->adc_endstop) {
        adc_endstop_report_error(lc->adc_endstop, error_code);
    }
}

// Add a measurement to the buffer
static void
add_load_cell_data(struct load_cell *lc, uint32_t stime, uint32_t mtime
               , int32_t sample)
{
    uint32_t tdiff = mtime - stime;
    if (lc->time_shift)
        tdiff = (tdiff + (1<<(lc->time_shift - 1))) >> lc->time_shift;
    if (tdiff >= TCODE_ERROR) {
        add_load_cell_error(lc, SE_SCHEDULE);
        return;
    }
    append_load_cell_measurement(lc, tdiff, sample);
    // endstop is optional, report if enabled
    if (lc->adc_endstop) {
        adc_endstop_report_sample(lc->adc_endstop, sample, mtime);
    }
}

// ads1263 sensor query
static void
ads1263_query(struct load_cell *lc, uint32_t stime)
{
    uint8_t msg[7] = { 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint32_t measurement_time = timer_read_time();
    spidev_transfer(lc->spi, 1, sizeof(msg), msg);
    uint32_t read_end_time = timer_read_time();

    // check the status byte to see if the data is fresh, if not ignore
    if ((msg[1] & 0x40) == 0) {
        add_load_cell_error(lc, SE_DUPELICATE);
        return;
    }
    else if (read_end_time - measurement_time > MAX_SPI_READ_TIME) {
        add_load_cell_error(lc, SE_SPI_TIME);
        return;
    }
    // TODO: perform CRC check
    // uint8_t crc = msg[6]
    // else if (!test_crc(msg, crc))
    //   add_load_cell_error(lc, SE_CRC);

    int32_t sample = (msg[2] << 24) | (msg[3] << 16) | (msg[4] << 8) | msg[5];
    add_load_cell_data(lc, stime, measurement_time, sample);
}

// start/stop capturing load cell data
void
command_query_load_cell(uint32_t *args)
{
    uint8_t oid = args[0];
    struct load_cell *lc = oid_lookup(oid, command_config_load_cell);

    sched_del_timer(&lc->timer);
    lc->flags = 0;
    if (!args[2]) {
        // End measurements
        if (lc->adc_endstop) {
            adc_endstop_source_stopped(lc->adc_endstop);
        }
        if (lc->data_count)
            load_cell_report(lc, oid);
        sendf("load_cell_end oid=%c sequence=%hu", oid, lc->sequence);
        return;
    }
    // Start new measurements query
    lc->timer.waketime = args[1];
    lc->rest_ticks = args[2];
    lc->sequence = 0;
    lc->data_count = 0;
    lc->time_shift = args[3];
    sched_add_timer(&lc->timer);
}
DECL_COMMAND(command_query_load_cell,
             "query_load_cell oid=%c clock=%u rest_ticks=%u time_shift=%c");

// Background task that performs measurements
void
load_cell_capture_task(void)
{
    if (!sched_check_wake(&wake_load_cell))
        return;
    uint8_t oid;
    struct load_cell *lc;
    foreach_oid(oid, lc, command_config_load_cell) {
        uint_fast8_t flags = lc->flags;
        if (!(flags & SA_PENDING))
            continue;
        irq_disable();
        uint32_t stime = lc->timer.waketime;
        uint_fast8_t overflow = lc->overflow;
        lc->flags = 0;
        lc->overflow = 0;
        irq_enable();
        stime -= lc->rest_ticks;
        while (overflow--) {
            add_load_cell_error(lc, SE_OVERFLOW);
            flush_if_full(lc, oid);
        }
        uint_fast8_t chip = lc->chip_type;
        if (chip == CHIP_ADS1263) {
            ads1263_query(lc, stime);
        }
        flush_if_full(lc, oid);
    }
}
DECL_TASK(load_cell_capture_task);
