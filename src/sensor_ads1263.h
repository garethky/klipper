#ifndef __ADS_1263_H
#define __ADS_1263_H

#include <stdint.h> // uint8_t
#include "sensor_multiplex_adc.h" // struct mux_adc_sample

struct ads1263_sensor {
    struct spidev_s *spi;   // SPI bus
    struct gpio_in  drdy;   // pin used to test is a sample is ready
};
struct ads1263_sensor *ads1263_oid_lookup(uint8_t oid);
uint8_t ads1263_is_ready(struct ads1263_sensor *ads1263_sensor);
void ads1263_query(struct ads1263_sensor *ads1263_sensor
                    , struct mux_adc_sample *sample);

#endif // sensor_ads1263.h