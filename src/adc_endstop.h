#ifndef __ADC_EDNSTOP_H
#define __ADC_EDNSTOP_H

#include <stdint.h> // uint8_t

struct adc_endstop *adc_endstop_oid_lookup(uint8_t oid);
void adc_endstop_report_sample(struct adc_endstop *ae, int32_t sample
                                , uint32_t ticks);
void adc_endstop_report_error(struct adc_endstop *ae, uint8_t error_code);
void adc_endstop_source_stopped(struct adc_endstop *ae);

#endif // adc_endstop.h