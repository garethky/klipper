// Support for bit-banging commands to HX711 and HX717 ADC chips
//
// Copyright (C) 2023 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <complex.h>
#include <stdint.h>
#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_MACH_AVR
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_shutdown
#include "sensor_multi_hx71x.h" // hx71x_query

/****************************************************************
 * Timing
 ****************************************************************/

typedef unsigned int multi_hx71x_time_t;

static multi_hx71x_time_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

static inline int
multi_hx71x_check_elapsed(multi_hx71x_time_t t1, multi_hx71x_time_t t2
                       , multi_hx71x_time_t ticks)
{
    return t2 - t1 >= ticks;
}

// The AVR micro-controllers require specialized timing
#if CONFIG_MACH_AVR

#include <avr/interrupt.h> // TCNT1

static multi_hx71x_time_t
multi_hx71x_get_time(void)
{
    return TCNT1;
}

#define multi_hx71x_delay_no_irq(start, ticks) (void)(ticks)
#define multi_hx71x_delay(start, ticks) (void)(ticks)

#else

static multi_hx71x_time_t
multi_hx71x_get_time(void)
{
    return timer_read_time();
}

static inline void
multi_hx71x_delay_no_irq(multi_hx71x_time_t start, multi_hx71x_time_t ticks)
{
    while (!multi_hx71x_check_elapsed(start, multi_hx71x_get_time(), ticks))
        ;
}

static inline void
multi_hx71x_delay(multi_hx71x_time_t start, multi_hx71x_time_t ticks)
{
    while (!multi_hx71x_check_elapsed(start, multi_hx71x_get_time(), ticks))
        irq_poll();
}

#endif


/****************************************************************
 * HX711 and HX717 Support
 ****************************************************************/
#define MIN_PULSE_TIME  nsecs_to_ticks(200)
#define MAX_READ_TIME timer_from_us(50)

int8_t
multi_hx71x_is_ready(struct multi_hx71x_sensor *multi_hx71x) {
    for (uint8_t i = 0; i < multi_hx71x->chip_count; i++) {
        // if the pin is low the sample is ready
        if (gpio_in_read(multi_hx71x->dout[i]) == 1) {
            return 0;
        }
    }
    return 1;
}

// Pulse all clock pins to move to the next bit
inline static void
multi_hx71x_pulse_clocks(struct multi_hx71x_sensor *multi_hx71x) {
    irq_disable();
    for (uint8_t i = 0; i < multi_hx71x->chip_count; i++) {
        gpio_out_write(multi_hx71x->sclk[i], 1);
    }
    multi_hx71x_delay_no_irq(multi_hx71x_get_time(), MIN_PULSE_TIME);
    for (uint8_t i = 0; i < multi_hx71x->chip_count; i++) {
        gpio_out_write(multi_hx71x->sclk[i], 0);
    }
    irq_enable();
}

// hx71x ADC query
void
multi_hx71x_query(struct multi_hx71x_sensor *multi_hx71x,
                  struct mux_adc_sample *sample)
{
    // if any pin is high the samples are not all ready
    if (!multi_hx71x_is_ready(multi_hx71x)) {
        sample->sample_not_ready = 1;
        return;
    }

    // data is ready
    int32_t counts[4] = {0, 0, 0, 0};
    sample->measurement_time = timer_read_time();
    for (uint8_t sample_idx = 0; sample_idx < 24; sample_idx++) {
        multi_hx71x_pulse_clocks(multi_hx71x);
        multi_hx71x_delay(multi_hx71x_get_time(), MIN_PULSE_TIME);
        // read 2's compliment int bits
        for (uint8_t i = 0; i < multi_hx71x->chip_count; i++) {
            counts[i] = (counts[i] << 1) | gpio_in_read(multi_hx71x->dout[i]);
        }
    }

    // bit bang 1 to 4 more bits to configure gain & channel for the next sample
    for (uint8_t gain_idx = 0;
         gain_idx < multi_hx71x->gain_channel;
         gain_idx++) {
        multi_hx71x_pulse_clocks(multi_hx71x);
        if (gain_idx < multi_hx71x->gain_channel - 1) {
            multi_hx71x_delay(multi_hx71x_get_time(), MIN_PULSE_TIME);
        }
    }

    if ((timer_read_time() - sample->measurement_time) > MAX_READ_TIME) {
        // if this happens mcu is probably overloaded
        shutdown("Multi HX71x read timing error, read took too long");
    }

    for (uint8_t i = 0; i < multi_hx71x->chip_count; i++) {
        // extend 2's complement 24 bits to 32bits
        if (counts[i] >= 0x800000) {
            counts[i] |= 0xFF000000;
        }
        if (!gpio_in_read(multi_hx71x->dout[i]) || counts[i] < -0x7FFFFF
                || counts[i] > 0x7FFFFF) {
            // TODO: only shutdown if homing, else reset
            shutdown("Multi HX71x output error, suspected sensor desync" \
                     " or reboot");
        }
    }

    // sum loadcell readings
    // 4 x 24 bits always fits in 32 bits so this math is overflow safe:
    for (uint8_t i = 0; i < multi_hx71x->chip_count; i++) {
        sample->counts += counts[i];
    }
}

// Create an multi hx71x sensor
void
command_config_multi_hx71x(uint32_t *args)
{
    struct multi_hx71x_sensor *multi_hx71x = oid_alloc(args[0]
                , command_config_multi_hx71x, sizeof(*multi_hx71x));
    uint8_t chip_count = args[1];
    if (chip_count < 1 || chip_count > 4) {
        shutdown("Multi HX71x only supports 1 to 4 sensors");
    }
    multi_hx71x->chip_count = chip_count;
    uint8_t gain_channel = args[2];
    if (gain_channel < 1 || gain_channel > 4) {
        shutdown("Multi HX71x gain/channel out of range");
    }
    multi_hx71x->gain_channel = gain_channel;

    uint8_t arg_idx = 3;
    for (uint8_t chip_idx = 0; chip_idx < chip_count; chip_idx++) {
        multi_hx71x->dout[chip_idx] = gpio_in_setup(args[arg_idx], -1);
        multi_hx71x->sclk[chip_idx] = gpio_out_setup(args[arg_idx + 1], 0);
        arg_idx += 2;
    }
}
DECL_COMMAND(command_config_multi_hx71x, "config_multi_hx71x oid=%c" \
    " chip_count=%c gain_channel=%c dout0_pin=%u sclk0_pin=%u dout1_pin=%u" \
    " sclk1_pin=%u dout2_pin=%u sclk2_pin=%u dout3_pin=%u sclk3_pin=%u");

// Lookup a look up a multi hx71x sensor instance by oid
struct multi_hx71x_sensor *
multi_hx71x_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_multi_hx71x);
}
