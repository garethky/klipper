#ifndef __HX71X_H
#define __HX71X_H

#include <stdint.h> // uint8_t
#include "sensor_multiplex_adc.h" // struct mux_adc_sample

struct hx71x_sensor {
    struct gpio_in  dout;   // pin used to receive data from the hx71x
    struct gpio_out sclk;   // pin used to generate clock for the hx71x
    uint8_t gain_channel;   // the gain+channel selection (1-4)
};
struct hx71x_sensor *hx71x_oid_lookup(uint8_t oid);
int8_t hx71x_is_ready(struct hx71x_sensor *hx71x);
void hx71x_query(struct hx71x_sensor *hx71x_sensor
                            , struct mux_adc_sample *sample);

#endif // sensor_hx71x.h
