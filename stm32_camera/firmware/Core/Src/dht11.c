/*
 * DHT11 driver.
 *
 * Protocol: the MCU pulls DATA low for >= 18 ms, then releases it. The sensor
 * answers ~80 us low + ~80 us high, then sends 40 bits. Each bit is ~50 us low
 * followed by a high of ~27 us (0) or ~70 us (1). Bytes: humidity int, humidity
 * dec, temperature int, temperature dec, checksum (sum of the first four).
 *
 * Timing uses TIM2 as a free-running 1 MHz counter (1 tick = 1 us).
 */
#include <stdio.h>
#include "dht11.h"
#include "main.h"

#define DHT_PORT          GPIOE
#define DHT_PIN           GPIO_PIN_4
#define BIT_ONE_THRESHOLD 40U     /* us: high longer than this = 1 */

static dht11_reading_t last_ok;
static uint32_t last_ok_tick;
static int have_reading;

static inline uint32_t micros(void)
{
  return TIM2->CNT;
}

static inline int line_high(void)
{
  return (DHT_PORT->IDR & DHT_PIN) != 0;
}

/* Wait while the line is at 'level'; return how long it stayed there in us, or -1 after timeout_us. */
static int32_t wait_while(int level, uint32_t timeout_us)
{
  uint32_t start = micros();
  while (line_high() == level)
  {
    if (micros() - start > timeout_us)
      return -1;
  }
  return (int32_t)(micros() - start);
}

void dht11_init(void)
{
  GPIO_InitTypeDef g = {0};

  /* TIM2: 32-bit, clocked at 108 MHz (APB1 54 MHz x2), /108 = 1 MHz, free-running. */
  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM2->PSC = 107;
  TIM2->ARR = 0xFFFFFFFF;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->CR1 = TIM_CR1_CEN;

  /* PE4 open-drain: writing 1 releases the line, the module's pull-up holds it high.
   * The DHT11 runs from 5 V (the board's 3.3 V sags when the camera streams), so DATA
   * idles at 5 V. PE4 is 5 V tolerant (FT) only with the internal pull resistors off. */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
  g.Pin = DHT_PIN;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT_PORT, &g);
}

dht11_status_t dht11_read(dht11_reading_t *out)
{
  uint8_t data[5] = {0};

  /* Start signal: low for 20 ms, then release. */
  HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);

  /* Sensor response: line goes low within ~40 us, low ~80 us, high ~80 us. */
  if (wait_while(1, 200) < 0)
    return DHT11_NO_RESPONSE;
  if (wait_while(0, 120) < 0 || wait_while(1, 120) < 0)
    return DHT11_TIMEOUT;

  /* 40 data bits: ~50 us low, then a high whose length is the bit value. */
  for (int i = 0; i < 40; i++)
  {
    if (wait_while(0, 100) < 0)
      return DHT11_TIMEOUT;
    int32_t high_us = wait_while(1, 120);
    if (high_us < 0)
      return DHT11_TIMEOUT;
    data[i / 8] = (uint8_t)((data[i / 8] << 1) | (high_us > (int32_t)BIT_ONE_THRESHOLD));
  }

  if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4])
    return DHT11_BAD_CHECKSUM;

  out->humidity = data[0];
  out->humidity_dec = data[1];
  out->temperature = data[2];
  out->temperature_dec = data[3];
  last_ok = *out;
  last_ok_tick = HAL_GetTick();
  have_reading = 1;
  return DHT11_OK;
}

/* Diagnostic: send the start pulse, then record the DATA line for 6 ms and print the
 * edges. A working DHT11 gives ~84 edges (response + 40 bits); none = silent sensor. */
void dht11_probe(void)
{
  uint32_t edges = 0, t_first = 0, t_last = 0;
  uint32_t t0, now;
  int level, prev;
  uint16_t times[12];

  HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
  t0 = micros();
  prev = line_high();
  while ((now = micros()) - t0 < 6000)
  {
    level = line_high();
    if (level != prev)
    {
      if (edges < 12)
        times[edges] = (uint16_t)(now - t0);
      if (edges == 0)
        t_first = now - t0;
      t_last = now - t0;
      edges++;
      prev = level;
    }
  }
  printf("[dht11] probe: line %s after release, %lu edges in 6 ms",
         line_high() ? "high" : "LOW", (unsigned long)edges);
  if (edges)
  {
    printf(" (first at %lu us, last at %lu us; first edges us:", (unsigned long)t_first, (unsigned long)t_last);
    for (uint32_t i = 0; i < edges && i < 12; i++)
      printf(" %u", times[i]);
    printf(")");
  }
  printf("\r\n");
}

int dht11_last(dht11_reading_t *out, uint32_t *age_ms)
{
  if (!have_reading)
    return 0;
  *out = last_ok;
  *age_ms = HAL_GetTick() - last_ok_tick;
  return 1;
}

const char *dht11_status_text(dht11_status_t s)
{
  switch (s)
  {
    case DHT11_OK:           return "ok";
    case DHT11_NO_RESPONSE:  return "no response";
    case DHT11_TIMEOUT:      return "timeout";
    case DHT11_BAD_CHECKSUM: return "bad checksum";
  }
  return "?";
}
