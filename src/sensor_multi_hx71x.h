#ifndef __MULTI_HX71X_H
#define __MULTI_HX71X_H

#include <stdint.h> // uint8_t
#include "sensor_multiplex_adc.h" // struct mux_adc_sample

struct multi_hx71x_sensor {
    struct gpio_in  dout[4]; // pins used to receive data from the hx71x 0
    struct gpio_out sclk[4]; // pins used to generate clock for the hx71x 0
    uint8_t chip_count;      // the numbers of sensor chips, 3 or 4
    uint8_t gain_channel;    // the gain+channel selection (1-4)
};
struct multi_hx71x_sensor *multi_hx71x_oid_lookup(uint8_t oid);
int8_t multi_hx71x_is_ready(struct multi_hx71x_sensor *hx71x);
void multi_hx71x_query(struct multi_hx71x_sensor *multi_hx71x_sensor
                            , struct mux_adc_sample *sample);

#endif // sensor_multi_hx71x.h
