// Handling of end stops.
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // uint8_t
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "trsync.h" // trsync_do_trigger
#include "sched.h" // shutdown
#include "adc_endstop.h" //adc_endstop_report_sample

// Flags
enum { ESF_HOMING=1<<0, ESF_IS_TRIGGERED=1<<1 };

// Exponential Moving Average Filter
#define HALF_INT64 (UINT64_MAX / 2 + 1)
struct ema_filter {
    uint64_t state, half, unshift;
    int32_t average;
    uint8_t alpha;
};

void
ema_filter_init(struct ema_filter *ef, int32_t setpoint, uint8_t alpha) {
    ef->alpha = alpha;
    ef->half = (uint64_t) (alpha > 0 ? 1 << (alpha - 1) : 0);
    ef->unshift = HALF_INT64 >> ef->alpha;
    ef->state = (HALF_INT64 + ((uint64_t)(setpoint) << alpha) - setpoint);
}

static int32_t
ema_filter_update(struct ema_filter *ef, int32_t sample) {
    // TODO: This is _sus_ until tested and proven to work
    // expecially casting between types in c
    ef->state += (uint64_t)sample;
    uint64_t average = (ef->state + ef->half) >> ef->alpha;
    average -= ef->unshift;
    ef->state -= average;
    ef->average = (int32_t)average;
    return ef->average;
}

struct adc_endstop {
    uint32_t trigger_ticks, last_sample_ticks, deadband;
    int32_t last_sample, setpoint, trend_max, trend_min;
    uint8_t flags, trigger_count, trigger_reason, sample_count;
    struct trsync *ts;
    struct ema_filter *sample_filter;
    struct ema_filter *trend_filter;
};

static uint8_t
is_flag_set(uint8_t flag, uint8_t flags) {
    return !!(flag & flags);
}

static uint8_t
set_flag(uint8_t mask, uint8_t value, uint8_t flags) {
    if (!value) {
        return flags |= mask;
    } else {
        return flags &= mask;
    }
}

void
try_trigger(struct adc_endstop *ae, uint8_t is_homing) {
    ae->flags = set_flag(ae->flags, ESF_IS_TRIGGERED, 1);
    if (is_homing) {
        trsync_do_trigger(ae->ts, ae->trigger_reason);
        // Disable further triggering
        ae->flags = set_flag(ESF_HOMING, 0, ae->flags);
    }
}

// Used by ADC data sources to report new ADC sample data
void
adc_endstop_report_sample(struct adc_endstop *ae, int32_t sample
                        , uint32_t ticks)
{
    // save new sample
    ae->last_sample = sample;
    ae->last_sample_ticks = ticks;

    // Update filters and get averages:
    // TODO: int overflow is possible here
    int32_t sample_avg = ema_filter_update(ae->sample_filter, sample);
    int32_t trend_avg = ae->trend_filter->average;

    // Check for trigger
    uint8_t is_trigger = (sample_avg > (trend_avg + ae->deadband))
                        || (sample_avg < (trend_avg - ae->deadband));
    uint8_t is_homing = is_flag_set(ESF_HOMING, ae->flags);

    if (is_trigger && ae->trigger_count > 0) {
        // the first triggering sample sets the trigger time
        if (is_homing && ae->trigger_count == ae->sample_count) {
            ae->trigger_ticks = ticks;
        }
        ae->trigger_count -= 1;

        if (ae->trigger_count == 0) {
            try_trigger(ae, is_homing);
        }
    }
    else if (!is_trigger && ae->trigger_count < ae->sample_count) {
        // any sample inside the deadband resets the trigger
        ae->trigger_count = ae->sample_count;
        ae->flags = set_flag(ae->flags, ESF_IS_TRIGGERED, 0);
        // if homing, also clear the trigger time
        if (is_homing) {
            ae->trigger_ticks = 0;
        }
    }

    // Only update trend filter with non-trigger points
    if (!is_trigger) {
        trend_avg = ema_filter_update(ae->trend_filter, sample);
        // check for trend out of bounds
        if (trend_avg > ae->trend_max || trend_avg > ae->trend_min) {
            try_trigger(ae, is_homing);
        }
    }
}

void
adc_endstop_report_error(struct adc_endstop *ae, uint8_t error_code) {
    uint8_t is_homing = is_flag_set(ESF_HOMING, ae->flags);
    if (is_homing) {
        // ATTENTION: is this good form or should we _only_ call shutdown() ?
        // also how do we log the error in such a way that the user is sure to see it?
        trsync_do_trigger(ae->ts, ae->trigger_reason);
        shutdown("adc_endstop: Sensor reported an error while homing");
    }
}

void
adc_endstop_source_stopped(struct adc_endstop *ae) {
    uint8_t is_homing = is_flag_set(ESF_HOMING, ae->flags);
    if (is_homing) {
        // ATTENTION: is this good form or should we _only_ call shutdown() ?
        trsync_do_trigger(ae->ts, ae->trigger_reason);
        shutdown("adc_endstop: Sensor stopped sending data while homing");
    }
}

static void
reset_endstop(struct adc_endstop *ae, uint32_t deadband, uint32_t setpoint
            ,int32_t trend_min, int32_t trend_max
            , uint8_t sample_alpha, uint8_t trend_alpha) {
    ae->deadband = deadband;
    ae->setpoint = setpoint;
    ae->trend_min = trend_min;
    ae->trend_max = trend_max;
    ema_filter_init(ae->sample_filter, setpoint, sample_alpha);
    ema_filter_init(ae->trend_filter, setpoint, trend_alpha);
}

// Create the ADC endstop
void
command_config_adc_endstop(uint32_t *args)
{
    struct adc_endstop *ae = oid_alloc(args[0], command_config_adc_endstop
                                        , sizeof(*ae));
    reset_endstop(ae, args[1], args[2], args[3], args[4], args[5], args[6]);
}
DECL_COMMAND(command_config_adc_endstop, "config_adc_endstop oid=%c deadband=%u"
    " setpoint=%i trend_min=%i trend_max=%i sample_filter_alpha=%c"
    " trend_filter_alpha=%c");

// Lookup an adc_endstop
struct adc_endstop *
adc_endstop_oid_lookup(uint8_t oid) {
    return oid_lookup(oid, command_config_adc_endstop);
}

// Reset filters and triggering parameters
void
command_reset_adc_endstop(uint32_t *args)
{
    struct adc_endstop *ae = oid_lookup(args[0], command_config_adc_endstop);
    reset_endstop(ae, args[1], args[2], args[3], args[4], args[5], args[6]);
}
DECL_COMMAND(command_reset_adc_endstop, "reset_adc_endstop oid=%c deadband=%u"
    " setpoint=%i trend_min=%i trend_max=%i sample_filter_alpha=%c"
    " trend_filter_alpha=%c");

// Home an axis
void
command_adc_endstop_home(uint32_t *args)
{
    struct adc_endstop *ae = oid_lookup(args[0], command_config_adc_endstop);
    ae->ts = trsync_oid_lookup(args[1]);
    ae->trigger_reason = args[2];
    ae->sample_count = args[3];

    if (!ae->sample_count) {
        // Disable end stop checking
        ae->ts = NULL;
        ae->flags = set_flag(ESF_HOMING, 0, ae->flags);
        return;
    }
    ae->trigger_count = ae->sample_count;
    ae->flags = set_flag(ESF_HOMING, 1, ae->flags);
}
DECL_COMMAND(command_adc_endstop_home,
             "adc_endstop_home oid=%c trsync_oid=%c trigger_reason=%c"
             " sample_count=%c");

void
command_adc_endstop_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct adc_endstop *ae = oid_lookup(oid, command_config_adc_endstop);

    sendf("adc_endstop_state oid=%c homing=%c is_triggered=%c trigger_ticks=%u"
            " sample=%i ticks=%u"
            , oid, is_flag_set(ESF_HOMING, ae->flags), ae->trigger_ticks
            , is_flag_set(ESF_IS_TRIGGERED, ae->flags)
            , ae->last_sample, ae->last_sample_ticks);
}
DECL_COMMAND(command_adc_endstop_query_state, "adc_endstop_query_state oid=%c");
