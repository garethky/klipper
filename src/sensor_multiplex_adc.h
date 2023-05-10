#ifndef __SENSOR_MULTIPLEX_ADC_H
#define __SENSOR_MULTIPLEX_ADC_H

#include <stdint.h> // uint8_t

// struct to contain the results of a sensor being read
struct mux_adc_sample {
    uint32_t measurement_time;
    int32_t counts;
    uint8_t sample_not_ready;
    uint8_t crc_error;
    uint8_t timing_error;
};

#endif // sensor_load_cell.h