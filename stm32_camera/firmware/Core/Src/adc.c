/*
 * ADC1, register level, one conversion at a time (no DMA, no interrupts).
 *
 * VDD comes from the internal reference VREFINT: ST measures it at 3.3 V in the
 * factory and stores the reading at 0x1FF0F44A (STM32F74x datasheet), so
 * VDD = 3300 * cal / reading. Pin voltages are scaled by that measured VDD, which
 * matters here because the board's 3.3 V sags to ~2.5 V while the camera runs.
 */
#include "main.h"
#include "adc.h"

static void adc_init_once(void)
{
  static int done;

  if (done)
    return;
  done = 1;
  __HAL_RCC_ADC1_CLK_ENABLE();
  ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0 | ADC_CCR_TSVREFE; /* 108/4 = 27 MHz */
  ADC1->SMPR1 |= 7U << ADC_SMPR1_SMP17_Pos;   /* VREFINT: longest sample time (>= 10 us) */
  ADC1->SMPR2 |= 7U << ADC_SMPR2_SMP3_Pos;    /* PA3: long sample time, high-impedance sensors */
  ADC1->SQR1 = 0;                             /* one conversion per start */
  ADC1->CR2 = ADC_CR2_ADON;
  HAL_Delay(1);
}

static uint32_t adc_raw_avg(uint32_t channel)
{
  uint32_t sum = 0;

  adc_init_once();
  ADC1->SQR3 = channel;
  for (int i = 0; i < 16; i++)
  {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC))
    {
    }
    sum += ADC1->DR;
  }
  return sum / 16U;
}

uint32_t adc_vdd_mv(void)
{
  const uint16_t cal = *(const uint16_t *)0x1FF0F44AU;
  uint32_t raw = adc_raw_avg(ADC_CH_VREFINT);

  return raw ? 3300U * cal / raw : 0;
}

uint32_t adc_pin_mv(uint32_t channel)
{
  uint32_t vdd = adc_vdd_mv();
  return adc_raw_avg(channel) * vdd / 4095U;
}

/*
 * CO estimate (MiCS-5524, "RED" sensing element).
 *
 * The breakout's load resistor RL (10 kOhm) and the sensor resistance Rs form a
 * divider from 5 V: Vout = 5 V * RL / (Rs + RL), so Rs = RL * (5 V - Vout) / Vout.
 * Only the ratio Rs/R0 is used, so the exact RL cancels out.
 *
 * ppm = 4.385 * (Rs/R0)^-1.179 is the fit used by Seeed's MiCS-6814 library for the
 * same RED element. R0 is the clean-air resistance: the highest Rs seen after the
 * 3-minute warm-up (clean air = highest resistance), decaying slowly (~3 h time
 * constant) so the baseline can follow sensor drift. It is an estimate: the sensor
 * also reacts to alcohol, H2, NH3 and VOCs, and it is not a calibrated CO meter.
 */
#include <math.h>

#define CO_SUPPLY_MV  5000.0f
#define CO_RL_OHM     10000.0f

static co_reading_t co;
static uint32_t co_tick;
static int co_have;
static float r0;
static float v_avg;

uint32_t co_sensor_update(void)
{
  /* Warm sensor output is only ~50-100 mV here, so one-second readings jitter by
   * +-20 mV. Smooth over ~10 s (called once per second) before computing Rs, and
   * take the baseline from the smoothed value so a single noisy dip can't set it. */
  float raw = (float)(adc_pin_mv(ADC_CH_PA3) * CO_DIVIDER);
  v_avg = co_have ? v_avg + 0.1f * (raw - v_avg) : raw;
  float v = v_avg;
  float rs = v > 1.0f ? CO_RL_OHM * (CO_SUPPLY_MV - v) / v : 1e9f;

  co.mv = (uint32_t)v;
  co.rs_ohm = (uint32_t)rs;
  co.warm = HAL_GetTick() >= CO_WARMUP_MS;
  if (co.warm)
  {
    r0 *= 0.9999f;            /* called once per second: ~2.8 h decay */
    if (rs > r0)
      r0 = rs;
    co.ppm = 4.385f * powf(rs / r0, -1.179f);
  }
  co.r0_ohm = (uint32_t)r0;
  co_tick = HAL_GetTick();
  co_have = 1;
  return co.mv;
}

int co_sensor_last(co_reading_t *out, uint32_t *age_ms)
{
  if (!co_have)
    return 0;
  *out = co;
  *age_ms = HAL_GetTick() - co_tick;
  return 1;
}
