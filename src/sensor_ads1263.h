#ifndef __ADS_1263_H
#define __ADS_1263_H

#include <stdint.h> // uint8_t
#include "sensor_load_cell.h" // struct load_cell_sample

struct ads1263_sensor {
    struct spidev_s *spi;
};
struct ads1263_sensor *ads1263_oid_lookup(uint8_t oid);
void ads1263_query(struct ads1263_sensor *ads1263_sensor
                    , struct load_cell_sample *sample);

#endif // sensor_ads1263.h