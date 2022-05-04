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
#include "load_cell_endstop.h" //load_cell_endstop_report_sample

// Flags
enum { FLAG_IS_HOMING = 1 << 0, FLAG_IS_TRIGGERED = 1 << 1 };

// Exponential Moving Average Filter
struct ema_filter {
    int64_t state, half;
    int32_t average;
    uint8_t alpha;
};

void
ema_filter_init_alpha(struct ema_filter *ef, uint8_t alpha)
{
    ef->alpha = alpha;
    ef->half = 1 << (ef->alpha - 1);
    ef->average = 0;
    ef->state = 0;
}

void
ema_filter_init_setpoint(struct ema_filter *ef, int32_t setpoint)
{
    ef->average = setpoint;
    ef->state = (((int64_t)setpoint) << ef->alpha) - setpoint;
}

static int32_t
ema_filter_update(struct ema_filter *ef, int32_t sample)
{
    ef->state += sample;
    uint_fast8_t neg = ((ef->state < 0) ? 1 : 0);
    ef->average = (ef->state - neg + ef->half) >> ef->alpha;
    ef->state -= ef->average;
    return ef->average;
}

// Endstop Structure
struct load_cell_endstop {
    uint32_t trigger_ticks, last_sample_ticks, deadband, settling_count;
    int32_t last_sample, crash_max, crash_min, settling_average, settling_index;
    uint8_t flags, trigger_count, trigger_reason, sample_count;
    struct trsync *ts;
    struct ema_filter sample_filter, trend_filter;
};

int32_t load_cell_endstop_sample_avg(struct load_cell_endstop *lce)
{
    return lce->sample_filter.average;
}

int32_t load_cell_endstop_trend_avg(struct load_cell_endstop *lce)
{
    return lce->trend_filter.average;
}

static uint8_t
is_flag_set(uint8_t flag, uint8_t flags)
{
    return !!(flag & flags);
}

static uint8_t
set_flag(uint8_t mask, uint8_t value, uint8_t flags)
{
    if (!value) {
        return flags |= mask;
    } else {
        return flags &= mask;
    }
}

static int32_t
average(int32_t avg, int32_t input, int32_t index)
{
    return avg + ((input - avg) / (index + 1));
}

void
try_trigger(struct load_cell_endstop *lce, uint8_t is_homing)
{
    lce->flags = set_flag(lce->flags, FLAG_IS_TRIGGERED, 1);
    if (is_homing) {
        trsync_do_trigger(lce->ts, lce->trigger_reason);
        // Disable further triggering
        lce->flags = set_flag(FLAG_IS_HOMING, 0, lce->flags);
    }
}

// Used by Load Cell sensor to report new ADC sample data
void
load_cell_endstop_report_sample(struct load_cell_endstop *lce, int32_t sample
                        , uint32_t ticks)
{
    // save new sample
    lce->last_sample = sample;
    lce->last_sample_ticks = ticks;

    uint8_t is_settling = lce->settling_index < lce->settling_count;

    // initialize the filters and settling average
    if (lce->settling_index == 0) {
        ema_filter_init_setpoint(&(lce->sample_filter), sample);
        ema_filter_init_setpoint(&(lce->trend_filter), sample);
        lce->settling_average = 0;
    }

    // Update filter and get averages:
    int32_t sample_avg = ema_filter_update(&(lce->sample_filter), sample);
    int32_t trend_avg = lce->trend_filter.average;

    // check for a crash and immediatly trigger
    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lce->flags);
    uint8_t is_crash =  (sample_avg > lce->crash_max)
                        || (sample_avg < lce->crash_min);
    uint8_t is_trigger = (sample_avg > (trend_avg + lce->deadband))
                        || (sample_avg < (trend_avg - lce->deadband));
    
    // always trigger for a suspected crash
    if (is_crash) {
        try_trigger(lce, is_homing);
    }
    // otherwise trigger if not settling
    else if (!is_settling && is_trigger && lce->trigger_count > 0) {
        // the first triggering sample sets the trigger time
        if (is_homing && lce->trigger_count == lce->sample_count) {
            lce->trigger_ticks = ticks;
        }
        lce->trigger_count -= 1;

        if (lce->trigger_count == 0) {
            try_trigger(lce, is_homing);
        }
    }
    else if (!is_trigger && lce->trigger_count < lce->sample_count) {
        // any sample inside the deadband resets the trigger count
        lce->trigger_count = lce->sample_count;
        lce->flags = set_flag(lce->flags, FLAG_IS_TRIGGERED, 0);
        // if homing, also clear the trigger time
        if (is_homing) {
            lce->trigger_ticks = 0;
        }
    }
    
    // continue settling
    if (is_settling) {
        lce->settling_average = average(lce->settling_average, sample
                                        , lce->settling_index);
        lce->settling_index += 1;
        // during setting, keep updating the trend filter so the
        // settling process is observable as the trend_filter average
        ema_filter_init_setpoint(&(lce->trend_filter), lce->settling_average);
    }
    else if (!is_trigger) {
        // Only update trend filter with non-trigger points
        trend_avg = ema_filter_update(&(lce->trend_filter), sample);
    }
}

void
load_cell_endstop_report_error(struct load_cell_endstop *lce, uint8_t error_code)
{
    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lce->flags);
    if (is_homing) {
        // ATTENTION: is this good form or should we _only_ call shutdown() ?
        //how do we log the error in such a way that the user is sure to see it?
        trsync_do_trigger(lce->ts, lce->trigger_reason);
        shutdown("load_cell_endstop: Sensor reported an error while homing");
    }
}

void
load_cell_endstop_source_stopped(struct load_cell_endstop *lce)
{
    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lce->flags);
    if (is_homing) {
        // ATTENTION: is this good form or should we _only_ call shutdown() ?
        trsync_do_trigger(lce->ts, lce->trigger_reason);
        shutdown("load_cell_endstop: Sensor stopped sending data while homing");
    }
}

static void
reset_endstop(struct load_cell_endstop *lce, uint32_t deadband
            , int32_t crash_min, int32_t crash_max, uint8_t sample_alpha
            , uint8_t trend_alpha, uint32_t settling_count)
{
    lce->deadband = deadband;
    lce->crash_min = crash_min;
    lce->crash_max = crash_max;
    lce->settling_count = settling_count > 1 ? settling_count : 1;
    lce->settling_index = lce->settling_average = 0;
    ema_filter_init_alpha(&(lce->sample_filter), sample_alpha);
    ema_filter_init_alpha(&(lce->trend_filter), trend_alpha);
}

// Create the Load Cell Endstop
void
command_config_load_cell_endstop(uint32_t *args)
{
    struct load_cell_endstop *lce = oid_alloc(args[0]
                            , command_config_load_cell_endstop, sizeof(*lce));
    reset_endstop(lce, args[1], args[2], args[3], args[4], args[5], args[6]);
}
DECL_COMMAND(command_config_load_cell_endstop, "config_load_cell_endstop oid=%c"
    " deadband=%i crash_min=%i crash_max=%i sample_filter_alpha=%c"
    " trend_filter_alpha=%c settling_count=%i");

// Lookup an load_cell_endstop
struct load_cell_endstop *
load_cell_endstop_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_load_cell_endstop);
}

// Reset filters and triggering parameters
void
command_reset_load_cell_endstop(uint32_t *args)
{
    struct load_cell_endstop *lce = oid_lookup(args[0]
                                        , command_config_load_cell_endstop);
    reset_endstop(lce, args[1], args[2], args[3], args[4], args[5], args[6]);
}
DECL_COMMAND(command_reset_load_cell_endstop, "reset_load_cell_endstop oid=%c"
    " deadband=%i crash_min=%i crash_max=%i sample_filter_alpha=%c"
    " trend_filter_alpha=%c settling_count=%i");

// Home an axis
void
command_load_cell_endstop_home(uint32_t *args)
{
    struct load_cell_endstop *lce = oid_lookup(args[0]
                                        , command_config_load_cell_endstop);
    lce->ts = trsync_oid_lookup(args[1]);
    lce->trigger_reason = args[2];
    lce->sample_count = args[3];

    if (!lce->sample_count) {
        // Disable end stop checking
        lce->ts = NULL;
        lce->flags = set_flag(FLAG_IS_HOMING, 0, lce->flags);
        return;
    }
    lce->trigger_count = lce->sample_count;
    lce->flags = set_flag(FLAG_IS_HOMING, 1, lce->flags);
}
DECL_COMMAND(command_load_cell_endstop_home,
             "load_cell_endstop_home oid=%c trsync_oid=%c trigger_reason=%c"
             " sample_count=%c");

void
command_load_cell_endstop_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct load_cell_endstop *lce = oid_lookup(oid
                                    , command_config_load_cell_endstop);

    sendf("load_cell_endstop_state oid=%c homing=%c is_triggered=%c"
            " trigger_ticks=%u sample=%i ticks=%u sample_avg=%i trend_avg=%i"
            , oid, is_flag_set(FLAG_IS_HOMING, lce->flags), lce->trigger_ticks
            , is_flag_set(FLAG_IS_TRIGGERED, lce->flags), lce->last_sample
            , lce->last_sample_ticks, lce->sample_filter.average
            , lce->trend_filter.average);
}
DECL_COMMAND(command_load_cell_endstop_query_state
                , "load_cell_endstop_query_state oid=%c");
