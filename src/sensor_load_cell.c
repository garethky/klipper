// Support Load Cell behaviour from various ADCs
//
// Copyright (C) 2023  Gareth Farrigton <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/misc.h" // timer_read_time
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "load_cell_endstop.h" // load_cell_endstop_report_sample
#include "sensor_ads1263.h" // ads1263_query
#include "sensor_hx71x.h" // hx711_query

enum { SENSOR_ADS1263, SENSOR_HX71X, SENSOR_ENUM_MAX };

DECL_ENUMERATION("load_cell_sensor_type", "ads1263", SENSOR_ADS1263);
DECL_ENUMERATION("load_cell_sensor_type", "HX711", SENSOR_HX71X);

enum { TCODE_ERROR = 0xff };

// Sample Error Types
enum {
    SE_OVERFLOW, SE_SCHEDULE, SE_SPI_TIME, SE_CRC, SE_DUPLICATE
};

// Flag types
enum {
    FLAG_PENDING = 1<<0
};

// Wait Types
enum {
    WAIT_NEXT_SAMPLE, WAIT_SAMPLE_DUPLICATE
};

#define SAMPLE_WIDTH 5
#define ERROR_WIDTH 2
#define SAMPLE_NOT_READY_DELAY timer_from_us(200)

struct load_cell {
    struct timer timer;
    struct load_cell_endstop *load_cell_endstop;
    struct load_cell_sample *sample;
    // ticks between samples, determined by the sample rate of the sensor
    uint32_t sample_interval_ticks;
    // time to wait for the next wakeup
    uint32_t rest_ticks;
    uint16_t sequence;
    uint8_t flags, sensor_type, sensor_oid, data_count, time_shift, overflow;
    // dont try to send anything larger than 64 bytes over the USB bus
    uint8_t data[SAMPLE_WIDTH * 3];
};

static struct task_wake wake_load_cell;

// Event handler that wakes load_cell_capture_task() periodically
static uint_fast8_t
load_cell_event(struct timer *timer)
{
    struct load_cell *lc = container_of(timer, struct load_cell, timer);
    uint8_t flags = lc->flags;
    if (lc->flags & FLAG_PENDING)
        lc->overflow++;
    else
        lc->flags = flags | FLAG_PENDING;
    sched_wake_task(&wake_load_cell);
    lc->timer.waketime += lc->rest_ticks;
    return SF_RESCHEDULE;
}

// Report local measurement buffer
static void
load_cell_report(struct load_cell *lc, uint8_t oid)
{
    sendf("load_cell_data oid=%c sequence=%hu data=%*s"
          , oid, lc->sequence, lc->data_count, lc->data);
    lc->data_count = 0;
    lc->sequence++;
}

// Add an entry to the measurement buffer
static void
append_load_cell_measurement(struct load_cell *lc, uint_fast8_t tcode
                            , uint_fast32_t data)
{
    lc->data[lc->data_count] = tcode;
    // raw adc counts
    lc->data[lc->data_count + 1] = data;
    lc->data[lc->data_count + 2] = data >> 8;
    lc->data[lc->data_count + 3] = data >> 16;
    lc->data[lc->data_count + 4] = data >> 24;
    lc->data_count += SAMPLE_WIDTH;
}

// Add an error to the measurement buffer
static void
add_load_cell_error(struct load_cell *lc, uint8_t error_code)
{
    // endstop is optional, report if enabled
    if (lc->load_cell_endstop) {
        load_cell_endstop_report_error(lc->load_cell_endstop, error_code);
    }

    lc->data[lc->data_count] = TCODE_ERROR;
    lc->data[lc->data_count + 1] = error_code;
    lc->data_count += ERROR_WIDTH;
}

// Add a measurement to the buffer
static void
add_load_cell_data(struct load_cell *lc, uint32_t stime, uint32_t mtime
               , int32_t sample)
{
    uint32_t tdiff = mtime - stime;
    if (lc->time_shift) {
        tdiff = (tdiff + (1<<(lc->time_shift - 1))) >> lc->time_shift;
    }

    if (tdiff >= TCODE_ERROR) {
        add_load_cell_error(lc, SE_SCHEDULE);
        return;
    }

    // endstop is optional, report if enabled
    if (lc->load_cell_endstop) {
        load_cell_endstop_report_sample(lc->load_cell_endstop, sample
                                            , mtime);
    }

    append_load_cell_measurement(lc, tdiff, sample);
}

// use the contents of the sample struct to report
static void 
add_sensor_result(struct load_cell_sample *sample) {
    // publish the results of the read
    if (sample->timing_error == 1) {
        add_load_cell_error(lc, SE_SPI_TIME);
    } else if (sample->crc_error == 1) {
        add_load_cell_error(lc, SE_CRC);
    } else if (sample->is_duplicate == 1) {
        add_load_cell_error(lc, SE_DUPLICATE);
    } else {
        add_load_cell_data(lc, stime, sample->measurement_time, sample->counts);
    }
}

void read_sensor(struct load_cell *lc) {
    // clear sample container
    struct load_cell_sample *sample = lc->sample;
    sample->timing_error = 0;
    sample->crc_error = 0;
    sample->is_duplicate = 0;
    sample->measurement_time = 0;
    sample->counts = 0;

    // read from the configured sensor
    if (lc->sensor_type == SENSOR_ADS1263) {
        struct ads1263_sensor *ads = ads1263_oid_lookup(lc->sensor_oid);
        ads1263_query(ads, sample);
    } else if (lc->sensor_type == SENSOR_HX71X) {
        struct hx71x_sensor *hx = hx71x_oid_lookup(lc->sensor_oid);
        hx71x_query(hx, sample);
    }
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

void
command_config_load_cell(uint32_t *args)
{
    uint8_t sensor_type = args[1];
    if (sensor_type >= SENSOR_ENUM_MAX) {
        shutdown("Invalid load cell sensor type");
    }
    struct load_cell *lc = oid_alloc(args[0], command_config_load_cell
                                     , sizeof(*lc));
    lc->timer.func = load_cell_event;
    lc->sensor_type = sensor_type;
    uint8_t lce_oid = args[2];
    // optional endstop
    if (lce_oid != 0) {
        lc->load_cell_endstop = load_cell_endstop_oid_lookup(lce_oid);
    }
}
DECL_COMMAND(command_config_load_cell,
             "config_load_cell oid=%c sensor_type=%c senor_oid=%c"
             " load_cell_endstop_oid=%c");

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
        if (lc->load_cell_endstop) {
            load_cell_endstop_source_stopped(lc->load_cell_endstop);
        }
        if (lc->data_count) {
            load_cell_report(lc, oid);
        }
        sendf("load_cell_end oid=%c sequence=%hu", oid, lc->sequence);
        return;
    }
    // Start new measurements query
    lc->timer.waketime = args[1];
    lc->sample_interval_ticks = args[2];
    lc->rest_ticks = SAMPLE_NOT_READY_DELAY;
    lc->time_shift = args[3];
    lc->sequence = 0;
    lc->data_count = 0;

    // this reads the sensor and clears its "ready" state
    // the next read should be a duplicate allowing for closer timing
    read_sensor(lc, );
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
        if (!(flags & FLAG_PENDING))
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
        read_sensor(lc);
        add_sensor_result(lc);
        flush_if_full(lc, oid);

        // if a sample was not ready, poll more frequently until one is
        if (lc->sample->is_duplicate == 1) {
            lc->rest_ticks = SAMPLE_NOT_READY_DELAY;
        } else {
            lc->rest_ticks = lc->sample_interval_ticks;
        }
    }
}
DECL_TASK(load_cell_capture_task);
