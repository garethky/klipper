#ifndef __SENSOR_MULTIPLEX_ADC_H
#define __SENSOR_MULTIPLEX_ADC_H

#include <stdint.h> // uint8_t

// struct to contain the results of a sensor being read
struct mux_adc_sample {
    uint32_t measurement_time;
    int32_t counts;
    uint8_t sample_not_ready;
};

#endif // sensor_load_cell.h
