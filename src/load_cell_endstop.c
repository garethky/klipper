// Load Cell based end stops.
//
// Copyright (C) 2023  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // uint8_t
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "trsync.h" // trsync_do_trigger
#include "sched.h" // shutdown
#include "load_cell_endstop.h" //load_cell_endstop_report_sample

// TODO
// Add watchdog timer that faults after 2 sample periods with no new sample

const uint8_t DEFAULT_SAMPLE_COUNT = 2;

// Flags
enum { FLAG_IS_HOMING = 1 << 0, FLAG_IS_TRIGGERED = 1 << 1
    , FLAG_IS_HOMING_TRIGGER = 1 << 2 };

// Endstop Structure
struct load_cell_endstop {
    uint32_t trigger_ticks, last_sample_ticks;
    int32_t last_sample, trigger_counts_min, trigger_counts_max, tare_counts;
    uint8_t flags, trigger_count, trigger_reason, sample_count;
    struct trsync *ts;
};

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
try_trigger(struct load_cell_endstop *lce, uint8_t is_homing
                , uint8_t is_homing_triggered)
{
    // set live trigger flag
    lce->flags = set_flag(FLAG_IS_TRIGGERED, 1, lce->flags);

    if (is_homing && !is_homing_triggered) {
        // this flag latches until a reset, disabling further triggering
        lce->flags = set_flag(FLAG_IS_HOMING_TRIGGER, 1, lce->flags);
        trsync_do_trigger(lce->ts, lce->trigger_reason);
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
            try_trigger(lce, is_homing, is_homing_triggered);
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

void
load_cell_endstop_report_error(struct load_cell_endstop *lce
                                    , uint8_t error_code)
{
    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lce->flags);
    if (is_homing) {
        if (error_code == 0) {
            shutdown("load_cell_endstop: "
            "Sensor reported an error while homing: SE_OVERFLOW");
        }
        else if (error_code == 1) {
            shutdown("load_cell_endstop: "
            "Sensor reported an error while homing: SE_SCHEDULE");
        }
        else if (error_code == 2) {
            shutdown("load_cell_endstop: "
            "Sensor reported an error while homing: SE_SPI_TIME");
        }
        else if (error_code == 3) {
            shutdown("load_cell_endstop: "
            "Sensor reported an error while homing: SE_CRC");
        }
        else if (error_code == 4) {
            return;  // sample_not_ready errors are OK
        }
        else {
            shutdown("load_cell_endstop: "
            "Sensor reported an error while homing: UNKNOWN");
        }
    }
}

void
load_cell_endstop_source_stopped(struct load_cell_endstop *lce)
{
    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lce->flags);
    if (is_homing) {
        shutdown("load_cell_endstop: Sensor stopped sending data while homing");
    }
}

static void
reset_endstop(struct load_cell_endstop *lce)
{
    lce->flags = 0;
    lce->trigger_count = lce->sample_count = DEFAULT_SAMPLE_COUNT;
    lce->trigger_ticks = 0;
    lce->trigger_counts_max = 0;
    lce->trigger_counts_min = 0;
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
    reset_endstop(lce);
}
DECL_COMMAND(command_config_load_cell_endstop, "config_load_cell_endstop"
                                               " oid=%c");

// Lookup a load_cell_endstop
struct load_cell_endstop *
load_cell_endstop_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_load_cell_endstop);
}

// Reset endstop
void
command_reset_load_cell_endstop(uint32_t *args)
{
    struct load_cell_endstop *lce = load_cell_endstop_oid_lookup(args[0]);
    reset_endstop(lce);
    set_endstop_range(lce, args[1], args[2]);
}
DECL_COMMAND(command_reset_load_cell_endstop, "reset_load_cell_endstop"
                " oid=%c trigger_counts=%u tare_counts=%i");

// Home an axis
void
command_load_cell_endstop_home(uint32_t *args)
{
    struct load_cell_endstop *lce = load_cell_endstop_oid_lookup(args[0]);
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
    lce->sample_count = args[3];
    set_endstop_range(lce, args[4], args[5]);
    lce->flags = set_flag(FLAG_IS_HOMING, 1, lce->flags);
}
DECL_COMMAND(command_load_cell_endstop_home,
             "load_cell_endstop_home oid=%c trsync_oid=%c trigger_reason=%c"
             " sample_count=%c trigger_counts=%u tare_counts=%i");

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
