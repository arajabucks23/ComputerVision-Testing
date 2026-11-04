/*
 * WS2812B driver: TIM1 channel 1 PWM on PE9, one PWM period per data bit,
 * the duty cycle of every period loaded by DMA.
 *
 *   TIM1 clock 216 MHz (APB2 108 MHz x2), period 270 ticks = 1.25 us (800 kHz).
 *   0 bit: high 0.35 us (76 ticks), 1 bit: high 0.70 us (151 ticks).
 *   These fit both the original WS2812B datasheet (T0H 0.25-0.55, T1H 0.65-0.95 us)
 *   and the newer one (T0H 0.22-0.38, T1H 0.58-1.0 us).
 *
 * DMA2 Stream 5 Channel 6 = TIM1_UP. Each update event (start of a bit) writes the
 * next duty value into CCR1's preload register, which takes effect at the next
 * update. DMA2 Stream 1 belongs to the camera (DCMI); TIM1_CH1's own DMA request
 * is only on Streams 1, 3 and 6, the update request avoids depending on a compare
 * match when the duty is 0.
 *
 * The timer runs all the time with CCR1 = 0, so PE9 is always driven low between
 * frames (never floating into the level shifter). The DMA buffer ends with a 0, so
 * the line returns low after the last bit and stays there; the >280 us reset gap
 * the newer WS2812B needs comes from ws2812_show() refusing to start a new frame
 * less than 2 ms after the previous one finished.
 *
 * The DMA buffer is in SRAM1 (section .led_dma, see the linker script). The DMA2
 * memory port reaches SRAM1 through the bus matrix, and the D-cache is off, so no
 * cache maintenance is needed.
 */
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "ws2812.h"

#define BITS_PER_LED    24U
#define DMA_LEN         (WS2812_NUM_LEDS * BITS_PER_LED + 1U)   /* + one 0 to end low */
#define RESET_GAP_MS    2U
#define FRAME_TIMEOUT_MS 20U    /* a frame takes 4.8 ms; longer means the DMA is stuck */

static uint16_t dma_buf[DMA_LEN] __attribute__((section(".led_dma"), aligned(32)));
static uint8_t fb[WS2812_NUM_LEDS][3];   /* G, R, B: the order the WS2812B expects */

static TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_ws2812;

static uint32_t tim_clk, period, t0h, t1h;
static volatile int busy;
static volatile uint32_t done_tick;
static uint32_t start_tick;
static volatile uint32_t frames_sent, errors;

static void dma_done(DMA_HandleTypeDef *h)
{
  (void)h;
  __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
  done_tick = HAL_GetTick();
  frames_sent++;
  busy = 0;
}

static void dma_error(DMA_HandleTypeDef *h)
{
  (void)h;
  __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
  TIM1->CCR1 = 0;
  done_tick = HAL_GetTick();
  errors++;
  busy = 0;
}

static uint32_t ns_to_ticks(uint32_t ns)
{
  return ((tim_clk / 1000000U) * ns + 500U) / 1000U;
}

void ws2812_init(void)
{
  GPIO_InitTypeDef g = {0};
  TIM_OC_InitTypeDef oc = {0};

  /* PE9 as a plain output, low, until the timer is driving low too. */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
  g.Pin = GPIO_PIN_9;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_PULLDOWN;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &g);

  /* APB2 timers run at 2x PCLK2 when the APB2 divider is not 1 (TIMPRE left at 0). */
  tim_clk = HAL_RCC_GetPCLK2Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0)
    tim_clk *= 2U;
  period = tim_clk / 800000U;
  t0h = ns_to_ticks(350);
  t1h = ns_to_ticks(700);

  __HAL_RCC_TIM1_CLK_ENABLE();
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = period - 1U;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    Error_Handler();

  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK)   /* also sets CCR1 preload */
    Error_Handler();
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)                /* 0 % duty: output low */
    Error_Handler();

  /* Hand PE9 to TIM1_CH1 (AF1) now that the timer output is low. */
  g.Mode = GPIO_MODE_AF_PP;
  g.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &g);

  __HAL_RCC_DMA2_CLK_ENABLE();
  hdma_ws2812.Instance = DMA2_Stream5;
  hdma_ws2812.Init.Channel = DMA_CHANNEL_6;
  hdma_ws2812.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_ws2812.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_ws2812.Init.MemInc = DMA_MINC_ENABLE;
  hdma_ws2812.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_ws2812.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_ws2812.Init.Mode = DMA_NORMAL;
  /* Very high: each value must reach CCR1 within one 1.25 us bit, even while the
   * camera's DMA2 Stream 1 (high priority) is moving a frame. */
  hdma_ws2812.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_ws2812.Init.FIFOMode = DMA_FIFOMODE_DISABLE;   /* direct mode: one value prefetched */
  if (HAL_DMA_Init(&hdma_ws2812) != HAL_OK)
    Error_Handler();
  hdma_ws2812.XferCpltCallback = dma_done;
  hdma_ws2812.XferErrorCallback = dma_error;

  /* Below the camera's DCMI/DMA interrupts (5): the LED interrupt only ends a frame. */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

  /* Explicitly turn every LED off: whatever the strip latched at power-up is cleared. */
  memset(dma_buf, 0, sizeof(dma_buf));
  ws2812_clear();
  done_tick = HAL_GetTick() - RESET_GAP_MS;
  ws2812_show();
  uint32_t t = HAL_GetTick();
  while (busy && HAL_GetTick() - t < FRAME_TIMEOUT_MS)
  {
  }
}

void ws2812_clear(void)
{
  memset(fb, 0, sizeof(fb));
}

void ws2812_set(uint32_t i, uint8_t r, uint8_t g, uint8_t b)
{
  if (i >= WS2812_NUM_LEDS)
    return;
  fb[i][0] = g > WS2812_MAX_LEVEL ? WS2812_MAX_LEVEL : g;
  fb[i][1] = r > WS2812_MAX_LEVEL ? WS2812_MAX_LEVEL : r;
  fb[i][2] = b > WS2812_MAX_LEVEL ? WS2812_MAX_LEVEL : b;
}

int ws2812_busy(void)
{
  if (busy && HAL_GetTick() - start_tick > FRAME_TIMEOUT_MS)
  {
    HAL_DMA_Abort(&hdma_ws2812);
    dma_error(&hdma_ws2812);
  }
  return busy;
}

int ws2812_show(void)
{
  if (ws2812_busy() || HAL_GetTick() - done_tick < RESET_GAP_MS)
    return -1;

  uint16_t *p = dma_buf;
  for (uint32_t i = 0; i < WS2812_NUM_LEDS; i++)
    for (uint32_t c = 0; c < 3; c++)
      for (uint8_t mask = 0x80; mask; mask >>= 1)
        *p++ = (uint16_t)((fb[i][c] & mask) ? t1h : t0h);
  *p = 0;

  busy = 1;
  start_tick = HAL_GetTick();
  if (HAL_DMA_Start_IT(&hdma_ws2812, (uint32_t)dma_buf, (uint32_t)&TIM1->CCR1, DMA_LEN) != HAL_OK)
  {
    busy = 0;
    errors++;
    return -1;
  }
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  return 0;
}

uint32_t ws2812_frames_sent(void)
{
  return frames_sent;
}

uint32_t ws2812_errors(void)
{
  return errors;
}

void ws2812_print_timing(void)
{
  printf("[led] TIM1 %lu MHz on PE9, bit %lu ticks (%lu ns), T0H %lu ticks (%lu ns), T1H %lu ticks (%lu ns)\r\n",
         (unsigned long)(tim_clk / 1000000U),
         (unsigned long)period, (unsigned long)(period * 1000U / (tim_clk / 1000000U)),
         (unsigned long)t0h, (unsigned long)(t0h * 1000U / (tim_clk / 1000000U)),
         (unsigned long)t1h, (unsigned long)(t1h * 1000U / (tim_clk / 1000000U)));
  printf("[led] %u LEDs, DMA2 Stream5 Ch6 (TIM1_UP), buffer %u bytes at 0x%08lX, max level %u/255\r\n",
         (unsigned)WS2812_NUM_LEDS, (unsigned)sizeof(dma_buf), (unsigned long)(uintptr_t)dma_buf,
         (unsigned)WS2812_MAX_LEVEL);
}
