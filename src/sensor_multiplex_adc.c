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
#include "sensor_multiplex_adc.h" // mux_adc_sample
#include "sensor_ads1263.h" // ads1263_query
#include "sensor_hx71x.h" // hx711_query

enum { SENSOR_ADS1263, SENSOR_HX71X, SENSOR_ENUM_MAX };

DECL_ENUMERATION("mux_adc_sensor_type", "ads1263", SENSOR_ADS1263);
DECL_ENUMERATION("mux_adc_sensor_type", "hx71x", SENSOR_HX71X);

enum { TCODE_ERROR = 0xff };

// Sample Error Types
enum {
    SE_OVERFLOW, SE_SCHEDULE, SE_READ_TIME, SE_CRC, SE_DUPLICATE
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

struct mux_adc {
    struct timer timer;
    // ticks between samples, determined by the sample rate of the sensor
    uint32_t sample_interval_ticks;
    // time to wait for the next wakeup
    uint32_t rest_ticks;
    uint16_t sequence;
    struct mux_adc_sample sample;
    // TODO: support 1 endstop for each channel on the ADC
    struct load_cell_endstop *load_cell_endstop;
    uint8_t flags, sensor_type, sensor_oid, data_count, time_shift, overflow;
    // dont try to send anything larger than 64 bytes over the USB bus
    uint8_t data[SAMPLE_WIDTH * 3];
};

static struct task_wake wake_multiplex_adc;

// Event handler that wakes mux_adc_capture_task() periodically
static uint_fast8_t
mux_adc_event(struct timer *timer)
{
    struct mux_adc *mux_adc = container_of(timer, struct mux_adc, timer);
    uint8_t flags = mux_adc->flags;
    if (mux_adc->flags & FLAG_PENDING)
        mux_adc->overflow++;
    else
        mux_adc->flags = flags | FLAG_PENDING;
    sched_wake_task(&wake_multiplex_adc);
    mux_adc->timer.waketime += mux_adc->rest_ticks;
    return SF_RESCHEDULE;
}

// Report local measurement buffer
static void
mux_adc_report(struct mux_adc *mux_adc, uint8_t oid)
{
    sendf("multiplex_adc_data oid=%c sequence=%hu data=%*s"
          , oid, mux_adc->sequence, mux_adc->data_count, mux_adc->data);
    mux_adc->data_count = 0;
    mux_adc->sequence++;
}

// Append an entry to the measurement buffer
static void
append_measurement(struct mux_adc *mux_adc, uint_fast8_t tcode
                            , uint_fast32_t data)
{
    mux_adc->data[mux_adc->data_count] = tcode;
    // raw adc counts
    mux_adc->data[mux_adc->data_count + 1] = data;
    mux_adc->data[mux_adc->data_count + 2] = data >> 8;
    mux_adc->data[mux_adc->data_count + 3] = data >> 16;
    mux_adc->data[mux_adc->data_count + 4] = data >> 24;
    mux_adc->data_count += SAMPLE_WIDTH;
}

// Append an error to the measurement buffer
static void
append_error(struct mux_adc *mux_adc, uint8_t error_code)
{
    // endstop is optional, report if enabled
    if (mux_adc->load_cell_endstop) {
        load_cell_endstop_report_error(mux_adc->load_cell_endstop, error_code);
    }

    mux_adc->data[mux_adc->data_count] = TCODE_ERROR;
    mux_adc->data[mux_adc->data_count + 1] = error_code;
    mux_adc->data_count += ERROR_WIDTH;
}

// Add a measurement to the buffer
static void
add_adc_data(struct mux_adc *mux_adc, uint32_t sample_time)
{
    uint32_t tdiff = mux_adc->sample.measurement_time - sample_time;
    if (mux_adc->time_shift) {
        tdiff = (tdiff + (1<<(mux_adc->time_shift - 1))) >> mux_adc->time_shift;
    }

    // if the time difference between when the sample was requested and 
    // when it was taken exceeds 0xFF its unusable
    if (tdiff >= TCODE_ERROR) {
        append_load_cell_error(mux_adc, SE_SCHEDULE);
        return;
    }

    // endstop is optional, report if enabled
    if (mux_adc->load_cell_endstop) {
        load_cell_endstop_report_sample(mux_adc->load_cell_endstop
                        , mux_adc->sample.counts
                        , mux_adc->sample.measurement_time);
    }

    append_load_cell_measurement(mux_adc, tdiff, mux_adc->sample.counts);
}

// use the contents of the sample container to report
static void 
add_sensor_result(struct mux_adc *mux_adc, uint32_t sample_time) {
    // publish the results of the read
    if (mux_adc->sample.timing_error) {
        append_error(mux_adc, SE_READ_TIME);
    } else if (mux_adc->sample.crc_error) {
        append_error(mux_adc, SE_CRC);
    } else if (mux_adc->sample.sample_not_ready) {
        append_error(mux_adc, SE_DUPLICATE);
    } else {
        add_load_cell_data(mux_adc, sample_time);
    }
}

void read_sensor(struct mux_adc *mux_adc) {
    // clear sample container
    mux_adc->sample.timing_error = 0;
    mux_adc->sample.crc_error = 0;
    mux_adc->sample.sample_not_ready = 0;
    mux_adc->sample.measurement_time = 0;
    mux_adc->sample.counts = 0;

    // read from the configured sensor
    if (mux_adc->sensor_type == SENSOR_ADS1263) {
        struct ads1263_sensor *ads = ads1263_oid_lookup(mux_adc->sensor_oid);
        ads1263_query(ads, &mux_adc->sample);
    } else if (mux_adc->sensor_type == SENSOR_HX71X) {
        struct hx71x_sensor *hx = hx71x_oid_lookup(mux_adc->sensor_oid);
        hx71x_query(hx, &mux_adc->sample);
    }
}

// Send load_cell_data message if buffer is full
static void
flush_if_full(struct mux_adc *mux_adc, uint8_t oid)
{
    if ((mux_adc->data_count + SAMPLE_WIDTH - 1) > ARRAY_SIZE(mux_adc->data)) {
        mux_adc_report(mux_adc, oid);
    }
}

void
command_config_multiplex_adc(uint32_t *args)
{
    struct mux_adc *mux_adc = oid_alloc(args[0], command_config_multiplex_adc
                                     , sizeof(*mux_adc));
    mux_adc->timer.func = mux_adc_event;
    mux_adc->sensor_type = args[1];
    if (mux_adc->sensor_type >= SENSOR_ENUM_MAX) {
        shutdown("Invalid multiplex adc sensor type");
    }
    mux_adc->sensor_oid = args[2];
    uint8_t mux_adc_oid = args[3];
    // optional endstop
    if (mux_adc_oid != 0) {
        mux_adc->load_cell_endstop = load_cell_endstop_oid_lookup(mux_adc_oid);
    }
}
DECL_COMMAND(command_config_multiplex_adc, "config_config_multiplex_adc oid=%c"
            " multiplex_adc_sensor_type=%c sensor_oid=%c"
            " load_cell_endstop_oid=%c");

// start/stop capturing ADC data
void
command_query_multiplex_adc(uint32_t *args)
{
    uint8_t oid = args[0];
    struct mux_adc *mux_adc = oid_lookup(oid, command_config_multiplex_adc);

    sched_del_timer(&mux_adc->timer);
    mux_adc->flags = 0;
    if (!args[2]) {
        // End measurements
        if (mux_adc->load_cell_endstop) {
            load_cell_endstop_source_stopped(mux_adc->load_cell_endstop);
        }
        if (mux_adc->data_count) {
            mux_adc_report(mux_adc, oid);
        }
        sendf("multiplex_adc_end oid=%c sequence=%hu", oid, mux_adc->sequence);
        return;
    }
    // Start new measurements query
    mux_adc->timer.waketime = args[1];
    mux_adc->sample_interval_ticks = args[2];
    mux_adc->rest_ticks = args[2];
    mux_adc->time_shift = args[3];
    mux_adc->sequence = 0;
    mux_adc->data_count = 0;

    sched_add_timer(&mux_adc->timer);
}
DECL_COMMAND(command_query_multiplex_adc,
             "query_multiplex_adc oid=%c clock=%u rest_ticks=%u time_shift=%c");

// Background task that performs measurements
void
multiplex_adc_capture_task(void)
{
    if (!sched_check_wake(&wake_multiplex_adc))
        return;
    uint8_t oid;
    struct mux_adc *mux_adc;
    foreach_oid(oid, mux_adc, command_config_multiplex_adc) {
        uint_fast8_t flags = mux_adc->flags;
        if (!(flags & FLAG_PENDING))
            continue;
        irq_disable();
        uint32_t sample_time = mux_adc->timer.waketime;
        uint_fast8_t overflow = mux_adc->overflow;
        mux_adc->flags = 0;
        mux_adc->overflow = 0;
        irq_enable();
        sample_time -= mux_adc->rest_ticks;
        while (overflow--) {
            append_error(mux_adc, SE_OVERFLOW);
            flush_if_full(mux_adc, oid);
        }
        read_sensor(mux_adc);
        add_sensor_result(mux_adc, sample_time);
        flush_if_full(mux_adc, oid);
        mux_adc->rest_ticks = mux_adc->sample_interval_ticks;
    }
}
DECL_TASK(multiplex_adc_capture_task);
