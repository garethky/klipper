// Support Load Cell behaviour from various ADCs
//
// Copyright (C) 2023  Gareth Farrigton <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_HAVE_GPIO_BITBANGING
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

// Flag types
enum {
    FLAG_PENDING = 1<<0
};

#define SAMPLE_WIDTH 4

struct mux_adc {
    struct timer timer;
    // time to wait for the next wakeup
    uint32_t rest_ticks;
    uint16_t sequence;
    struct mux_adc_sample sample;
    struct load_cell_endstop *load_cell_endstop;
    uint8_t flags, sensor_type, sensor_oid, data_count, overflow;
    // dont try to send anything larger than 48 bytes over the USB bus
    uint8_t data[SAMPLE_WIDTH * 12];
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

// Report local measurement buffer
static void
send_mux_adc_status(uint8_t oid, uint32_t mcu_time, uint32_t duration
                    , uint16_t next_sequence, uint8_t pending)
{
    sendf("multiplex_adc_status oid=%c clock=%u duration=%u"
            " next_sequence=%hu pending=%c",
            oid, mcu_time, duration, next_sequence, pending);
}

// Append an entry to the measurement buffer
static void
append_measurement(struct mux_adc *mux_adc, uint_fast32_t data)
{
    // raw adc counts
    mux_adc->data[mux_adc->data_count + 0] = data;
    mux_adc->data[mux_adc->data_count + 1] = data >> 8;
    mux_adc->data[mux_adc->data_count + 2] = data >> 16;
    mux_adc->data[mux_adc->data_count + 3] = data >> 24;
    mux_adc->data_count += SAMPLE_WIDTH;
}

// Add a measurement to the buffer
static void
add_adc_data(struct mux_adc *mux_adc)
{
    // endstop is optional, report if enabled
    if (mux_adc->load_cell_endstop) {
        load_cell_endstop_report_sample(mux_adc->load_cell_endstop
                        , mux_adc->sample.counts
                        , mux_adc->sample.measurement_time);
    }

    append_measurement(mux_adc, mux_adc->sample.counts);
}

// use the contents of the sample container to report
static void 
add_sensor_result(struct mux_adc *mux_adc) {
    if (mux_adc->sample.sample_not_ready) {
        return;
    }
    add_adc_data(mux_adc);
}

void read_sensor(struct mux_adc *mux_adc) {
    // clear sample container
    mux_adc->sample.sample_not_ready = 0;
    // set to saturation point so is nothing is read the value will be an error
    mux_adc->sample.counts = 0xFFFFFFFF;

    // read from the configured sensor
    if (CONFIG_HAVE_GPIO_SPI) {
        if (mux_adc->sensor_type == SENSOR_ADS1263) {
            struct ads1263_sensor *ads = ads1263_oid_lookup(mux_adc->sensor_oid);
            ads1263_query(ads, &mux_adc->sample);
            return;
        }
    }
    
    if (CONFIG_HAVE_GPIO_BITBANGING) {
        if (mux_adc->sensor_type == SENSOR_HX71X) {
            struct hx71x_sensor *hx = hx71x_oid_lookup(mux_adc->sensor_oid);
            hx71x_query(hx, &mux_adc->sample);
            return;
        }
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
    uint8_t lc_endstop_oid = args[3];
    // optional endstop
    if (lc_endstop_oid != 0) {
        mux_adc->load_cell_endstop = load_cell_endstop_oid_lookup(lc_endstop_oid);
    }
}
DECL_COMMAND(command_config_multiplex_adc, "config_multiplex_adc oid=%c"
            " mux_adc_sensor_type=%c sensor_oid=%c"
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
        if (mux_adc->data_count) {
            mux_adc_report(mux_adc, oid);
        }
        sendf("multiplex_adc_end oid=%c sequence=%hu", oid, mux_adc->sequence);
        return;
    }
    // Start new measurements query
    mux_adc->timer.waketime = args[1];
    mux_adc->rest_ticks = args[2];
    mux_adc->sequence = 0;
    mux_adc->data_count = 0;
    sched_add_timer(&mux_adc->timer);
}
DECL_COMMAND(command_query_multiplex_adc,
             "query_multiplex_adc oid=%c clock=%u rest_ticks=%u");

// report clock and queue status for clock sync
void
command_query_multiplex_adc_status(uint32_t *args)
{
    uint8_t oid = args[0];
    struct mux_adc *mux_adc = oid_lookup(oid, command_config_multiplex_adc);
    uint32_t mcu_time = timer_read_time();
    // move any pending measurement on sensor to queue so it is counted
    read_sensor(mux_adc);
    add_sensor_result(mux_adc);
    uint8_t pending = mux_adc->data_count / SAMPLE_WIDTH;
    uint32_t duration = mcu_time - timer_read_time();
    uint16_t next_sequence = mux_adc->sequence;
    // send back timing data
    send_mux_adc_status(oid, mcu_time, duration, next_sequence, pending);
    // send queue contents if full
    flush_if_full(mux_adc, oid);
}
DECL_COMMAND(command_query_multiplex_adc_status,
             "query_multiplex_adc_status oid=%c");

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
        mux_adc->flags = 0;
        read_sensor(mux_adc);
        add_sensor_result(mux_adc);
        flush_if_full(mux_adc, oid);
    }
}
DECL_TASK(multiplex_adc_capture_task);
