// Support for bit-banging commands to HX711 and HX717 ADC chips
//
// Copyright (C) 2023 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_MACH_AVR
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_shutdown
#include "sensor_hx71x.h" // hx71x_query

/****************************************************************
 * Timing
 ****************************************************************/

typedef unsigned int hx71x_time_t;

static hx71x_time_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

static inline int
hx71x_check_elapsed(hx71x_time_t t1, hx71x_time_t t2
                       , hx71x_time_t ticks)
{
    return t2 - t1 >= ticks;
}

// The AVR micro-controllers require specialized timing
#if CONFIG_MACH_AVR

#include <avr/interrupt.h> // TCNT1

static hx71x_time_t
hx71x_get_time(void)
{
    return TCNT1;
}

#define hx71x_delay_no_irq(start, ticks) (void)(ticks)
#define hx71x_delay(start, ticks) (void)(ticks)

#else

static hx71x_time_t
hx71x_get_time(void)
{
    return timer_read_time();
}

static inline void
hx71x_delay_no_irq(hx71x_time_t start, hx71x_time_t ticks)
{
    while (!hx71x_check_elapsed(start, hx71x_get_time(), ticks))
        ;
}

static inline void
hx71x_delay(hx71x_time_t start, hx71x_time_t ticks)
{
    while (!hx71x_check_elapsed(start, hx71x_get_time(), ticks))
        irq_poll();
}

#endif


/****************************************************************
 * HX711 and HX717 Support
 ****************************************************************/
#define MIN_PULSE_TIME  nsecs_to_ticks(200)
#define MAX_READ_TIME timer_from_us(50)

int8_t
hx71x_is_ready(struct hx71x_sensor *hx71x) {
    // if the pin is low the sample is ready
    return gpio_in_read(hx71x->dout) == 0;
}

// xh71x ADC query
void
hx71x_query(struct hx71x_sensor *hx71x, struct mux_adc_sample *sample)
{
    // if the pin is high the sample is not ready
    if (gpio_in_read(hx71x->dout)) {
        sample->sample_not_ready = 1;
        return;
    }

    // data is ready
    int32_t counts = 0x00;
    sample->measurement_time = timer_read_time();
    for (uint8_t sample_idx = 0; sample_idx < 24; sample_idx++) {
        irq_disable();
        gpio_out_write(hx71x->sclk, 1);
        hx71x_delay_no_irq(hx71x_get_time(), MIN_PULSE_TIME);
        gpio_out_write(hx71x->sclk, 0);
        irq_enable();
        hx71x_delay(hx71x_get_time(), MIN_PULSE_TIME);
        // read 2's compliment int bits
        counts = (counts << 1) | gpio_in_read(hx71x->dout);
    }

    // bit bang 1 to 4 more bits to configure gain & channel for the next sample
    for (uint8_t gain_idx = 0; gain_idx < hx71x->gain_channel; gain_idx++) {
        irq_disable();
        gpio_out_write(hx71x->sclk, 1);
        hx71x_delay_no_irq(hx71x_get_time(), MIN_PULSE_TIME);
        gpio_out_write(hx71x->sclk, 0);
        irq_enable();
        hx71x_delay(hx71x_get_time(), MIN_PULSE_TIME);
    }

    if ((timer_read_time() - sample->measurement_time) > MAX_READ_TIME) {
        // if this happens mcu is probably overloaded
        shutdown("HX71x read timing error, read took too long");
    }

    // extend 2's complement 24 bits to 32bits
    if (counts >= 0x800000) {
        counts |= 0xFF000000;
    }

    // The dout pin should be high (indicating not ready) after a read
    // if not the chip may have restarted and we have a desync
    if (!gpio_in_read(hx71x->dout) || counts < -0x7FFFFF || counts > 0x7FFFFF) {
        // TODO: only shutdown if homing, else reset
        shutdown("HX71x output error, suspected sensor desync or reboot");
    }

    sample->counts = counts;
}

// Create an hx71x sensor
void
command_config_hx71x(uint32_t *args)
{
    struct hx71x_sensor *hx71x = oid_alloc(args[0]
                            , command_config_hx71x, sizeof(*hx71x));
    hx71x->dout = gpio_in_setup(args[1], -1);
    hx71x->sclk = gpio_out_setup(args[2], 0);
    uint8_t gain_channel = args[3];
    if (gain_channel < 1 || gain_channel > 4) {
        shutdown("HX71x gain/channel out of range");
    }
    hx71x->gain_channel = gain_channel;
}
DECL_COMMAND(command_config_hx71x, "config_hx71x oid=%c dout_pin=%u" \
                                    " sclk_pin=%u gain_channel=%c");

// Lookup a look up an hx71x sensor instance by oid
struct hx71x_sensor *
hx71x_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_hx71x);
}
