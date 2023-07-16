// Load Cell based end stops.
//
// Copyright (C) 2023  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_COMMAND
#include "sched.h" // shutdown
#include "trsync.h" // trsync_do_trigger
#include "board/misc.h" // timer_read_time
#include "load_cell_endstop.h" //load_cell_endstop_report_sample

// Endstop Structure
struct load_cell_endstop {
    struct timer time;
    uint32_t trigger_ticks, last_sample_ticks, rest_ticks;
    struct trsync *ts;
    int32_t last_sample, trigger_counts_min, trigger_counts_max, tare_counts;
    uint8_t flags, sample_count, trigger_count, trigger_reason, watchdog_max
            , watchdog_count;
    
};

const uint8_t DEFAULT_SAMPLE_COUNT = 2;

// Flags
enum { FLAG_IS_HOMING = 1 << 0, FLAG_IS_TRIGGERED = 1 << 1
    , FLAG_IS_HOMING_TRIGGER = 1 << 2 };


static uint8_t
is_flag_set(uint8_t mask, uint8_t flags)
{
    return !!(mask & flags);
}

static uint8_t
set_flag(uint8_t mask, uint8_t value, uint8_t flags)
{
    if (value == 0) {
        return flags &= ~mask;
    } else {
        return flags |= mask;
    }
}

void
try_trigger(struct load_cell_endstop *lce)
{
    // set live trigger flag
    lce->flags = set_flag(FLAG_IS_TRIGGERED, 1, lce->flags);

    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lce->flags);
    uint8_t is_homing_triggered = is_flag_set(FLAG_IS_HOMING_TRIGGER,
                                                lce->flags);
    if (is_homing && !is_homing_triggered) {
        // this flag latches until a reset, disabling further triggering
        lce->flags = set_flag(FLAG_IS_HOMING_TRIGGER, 1, lce->flags);
        trsync_do_trigger(lce->ts, lce->trigger_reason);
    }
}

// Used by Multiplex ADC to report new raw ADC sample
void
load_cell_endstop_report_sample(struct load_cell_endstop *lce, int32_t sample
                        , uint32_t ticks)
{
    // save new sample
    lce->last_sample = sample;
    lce->last_sample_ticks = ticks;
    lce->watchdog_count = 0;

    uint8_t is_trigger = sample >= lce->trigger_counts_max 
                         || sample <= lce->trigger_counts_min;
    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lce->flags);
    uint8_t is_homing_triggered = is_flag_set(FLAG_IS_HOMING_TRIGGER,
                                                lce->flags);

    // update trigger state
    if (is_trigger && lce->trigger_count > 0) {
        // the first triggering sample when homing sets the trigger time
        if (is_homing && !is_homing_triggered 
                && lce->trigger_count == lce->sample_count) {
            lce->trigger_ticks = ticks;
        }

        lce->trigger_count -= 1;
        // when the trigger count hits zero, trigger the trsync
        if (lce->trigger_count == 0) {
            try_trigger(lce);
        }
    }
    else if (!is_trigger && lce->trigger_count < lce->sample_count) {
        // any sample thats not a trigger resets the trigger count
        lce->trigger_count = lce->sample_count;

        // "live" trigger view
        lce->flags = set_flag(FLAG_IS_TRIGGERED, 0, lce->flags);
        
        // if homing, but not yet triggered, clear the trigger time
        if (is_homing && !is_homing_triggered) {
            lce->trigger_ticks = 0;
        }
    }
}

// Timer callback that monitors for timeouts
static uint_fast8_t
watchdog_event(struct timer *t)
{
    struct load_cell_endstop *lce = container_of(t, struct load_cell_endstop
                                        , time);
    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lce->flags);
    uint8_t is_homing_trigger = is_flag_set(FLAG_IS_HOMING_TRIGGER, lce->flags);
    // the watchdog stops when not homing or when trsync becomes triggered
    if (!is_homing || is_homing_trigger) {
        return SF_DONE;
    }

    irq_disable();
    if (lce->watchdog_count > lce->watchdog_max) {
        shutdown("LoadCell Endstop timed out waiting on ADC data");
    }
    lce->watchdog_count += 1;
    irq_enable();

    // A sample was recently delivered, continue monitoring
    lce->time.waketime += lce->rest_ticks;
    return SF_RESCHEDULE;
}

static void
set_endstop_range(struct load_cell_endstop *lce, uint32_t trigger_counts
                , int32_t tare_counts)
{
    lce->tare_counts = tare_counts;
    lce->trigger_counts_max = tare_counts + trigger_counts;
    lce->trigger_counts_min = tare_counts - trigger_counts;
}

// Create a load_cell_endstop
void
command_config_load_cell_endstop(uint32_t *args)
{
    struct load_cell_endstop *lce = oid_alloc(args[0]
                            , command_config_load_cell_endstop, sizeof(*lce));
    lce->flags = 0;
    lce->trigger_count = lce->sample_count = DEFAULT_SAMPLE_COUNT;
    lce->trigger_ticks = 0;
    lce->trigger_counts_max = 0;
    lce->trigger_counts_min = 0;
    lce->watchdog_max = 0;
    lce->watchdog_count = 0;
}
DECL_COMMAND(command_config_load_cell_endstop, "config_load_cell_endstop"
                                               " oid=%c");

// Lookup a load_cell_endstop
struct load_cell_endstop *
load_cell_endstop_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_load_cell_endstop);
}

// Set the triggering range and tare value
void
command_set_range_load_cell_endstop(uint32_t *args)
{
    struct load_cell_endstop *lce = load_cell_endstop_oid_lookup(args[0]);
    set_endstop_range(lce, args[1], args[2]);
}
DECL_COMMAND(command_set_range_load_cell_endstop, "set_range_load_cell_endstop"
                " oid=%c trigger_counts=%u tare_counts=%i");

// Home an axis
void
command_load_cell_endstop_home(uint32_t *args)
{
    struct load_cell_endstop *lce = load_cell_endstop_oid_lookup(args[0]);
    sched_del_timer(&lce->time);
    lce->trigger_ticks = 0;
    // clear the homing trigger flag
    lce->flags = set_flag(FLAG_IS_HOMING_TRIGGER, 0, lce->flags);
    // 0 samples indicates homing is finished
    if (args[3] == 0) {
        // Disable end stop checking
        lce->ts = NULL;
        lce->flags = set_flag(FLAG_IS_HOMING, 0, lce->flags);
        return;
    }
    lce->ts = trsync_oid_lookup(args[1]);
    lce->trigger_reason = args[2];
    lce->time.waketime = args[3];
    lce->sample_count = args[4];
    lce->rest_ticks = args[5];
    lce->watchdog_max = args[6];
    lce->watchdog_count = 0;
    lce->flags = set_flag(FLAG_IS_HOMING, 1, lce->flags);
    lce->time.func = watchdog_event;
    
    sched_add_timer(&lce->time);
}
DECL_COMMAND(command_load_cell_endstop_home,
             "load_cell_endstop_home oid=%c trsync_oid=%c trigger_reason=%c"
             " clock=%u sample_count=%c rest_ticks=%u timeout=%u");

void
command_load_cell_endstop_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct load_cell_endstop *lce = load_cell_endstop_oid_lookup(args[0]);
    sendf("load_cell_endstop_state oid=%c homing=%c homing_triggered=%c"
        " is_triggered=%c trigger_ticks=%u sample=%i sample_ticks=%u"
            , oid, is_flag_set(FLAG_IS_HOMING, lce->flags)
            , is_flag_set(FLAG_IS_HOMING_TRIGGER, lce->flags)
            , is_flag_set(FLAG_IS_TRIGGERED, lce->flags), lce->trigger_ticks 
            , lce->last_sample, lce->last_sample_ticks);
}
DECL_COMMAND(command_load_cell_endstop_query_state
                , "load_cell_endstop_query_state oid=%c");
