/* ADC1 helpers: supply voltage (VREFINT) and analog inputs, in millivolts. */
#ifndef ADC_H
#define ADC_H

#include <stdint.h>

uint32_t adc_vdd_mv(void);                 /* board supply, from the factory-calibrated VREFINT */
uint32_t adc_pin_mv(uint32_t channel);     /* average of 16 conversions, scaled by the measured VDD */

#define ADC_CH_PA3       3U                /* CO sensor (MiCS-5524) output */
#define ADC_CH_VREFINT   17U

/* CO sensor (MiCS-5524 breakout): A0 goes through a 47k/47k divider to PA3, so the
 * sensor's own output is twice the PA3 voltage. Higher = more CO / reducing gas. */
#define CO_DIVIDER       2U

typedef struct
{
  uint32_t mv;          /* sensor output (A0), mV */
  uint32_t rs_ohm;      /* sensor resistance now */
  uint32_t r0_ohm;      /* clean-air baseline resistance (0 until warmed up) */
  float ppm;            /* estimated CO, valid when warm */
  int warm;             /* 0 during the first CO_WARMUP_MS after power-on */
} co_reading_t;

#define CO_WARMUP_MS     180000U           /* 3 minutes */

uint32_t co_sensor_update(void);           /* read now; returns sensor output in mV */
int co_sensor_last(co_reading_t *out, uint32_t *age_ms);   /* last reading; 0 if none yet */

#endif /* ADC_H */
