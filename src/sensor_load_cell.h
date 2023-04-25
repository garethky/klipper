#ifndef __SENSOR_LOAD_CELL_H
#define __SENSOR_LOAD_CELL_H

#include <stdint.h> // uint8_t

// struct to contain the results of a sensor being read
struct load_cell_sample {
    uint32_t measurement_time;
    int32_t counts;
    uint8_t is_duplicate;
    uint8_t crc_error;
    uint8_t timing_error;
};

#endif // sensor_load_cell.h