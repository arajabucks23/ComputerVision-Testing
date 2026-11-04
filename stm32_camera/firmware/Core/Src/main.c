/*
 * EvacAI camera node firmware: OV5640 -> NUCLEO-F746ZG -> Ethernet -> PC.
 *
 * Step 1 (this version): clocks, debug serial, LEDs and Ethernet with a static IP.
 *   - COM port (ST-LINK, 115200 baud) prints boot and link messages.
 *   - LD1 (green) blinks twice a second while the main loop runs.
 *   - The board answers ping on 192.168.10.2.
 * The camera stays powered down and in reset until step 2.
 */
#include <stdio.h>
#include "main.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "ethernetif.h"
#include "dht11.h"
#include "wiring_check.h"
#include "camera.h"
#include "ov5640.h"
#include "http_server.h"
#include "capture.h"
#include "adc.h"

UART_HandleTypeDef huart3;
static struct netif gnetif;

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART3_Init(void);
static void Netif_Config(void);

#define measure_vdd_mv adc_vdd_mv   /* supply voltage, see adc.c */

int main(void)
{
  /* Instruction cache on. The data cache stays OFF so DMA buffers (camera,
   * Ethernet) need no cache cleaning/invalidation. On the H743 the same
   * effect is done with an MPU "non-cacheable" region instead. */
  SCB_EnableICache();

  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  USART3_Init();
  setvbuf(stdout, NULL, _IONBF, 0);   /* printf goes out immediately */

  printf("\r\n[boot] EvacAI camera node, SYSCLK = %lu Hz\r\n", HAL_RCC_GetSysClockFreq());

  lwip_init();
  Netif_Config();
  http_server_init();
  printf("[boot] IP %s, log at http://192.168.10.2/log\r\n", ip4addr_ntoa(netif_ip4_addr(&gnetif)));

  /* DHT11 before anything else draws current (camera still off): tells apart
   * "already locked up when we started" from "locked up by the camera starting". */
  dht11_init();
  {
    dht11_reading_t r;
    while (HAL_GetTick() < 1100)   /* DHT11 needs 1 s after power-up */
    {
    }
    dht11_status_t s = dht11_read(&r);
    printf("[dht11] at boot, camera off: %s (VDD %lu mV)\r\n",
           s == DHT11_OK ? "ok" : dht11_status_text(s), (unsigned long)measure_vdd_mv());
  }

  /* Check every camera and DHT11 wire once at boot (press RESET to run it again). */
  wiring_check_run();

  printf("[power] VDD = %lu mV, camera off\r\n", (unsigned long)measure_vdd_mv());

  /* Sensor in JPEG mode at 640x480, then DCMI + DMA capture (steps 3 and 4). */
  int camera_ok = camera_start_jpeg(OV5640_R640x480) == 0;
  if (camera_ok)
  {
    camera_measure_sync(1000);     /* VSYNC/HREF check before DCMI takes the pins */
    capture_init();
    printf("[cam] streaming at http://192.168.10.2/stream\r\n");
    {
      dht11_reading_t r;
      HAL_Delay(1100);
      dht11_status_t s = dht11_read(&r);
      printf("[dht11] after camera start: %s (VDD %lu mV)\r\n",
             s == DHT11_OK ? "ok" : dht11_status_text(s), (unsigned long)measure_vdd_mv());
    }
  }

  uint32_t last_blink = 0, last_dht = HAL_GetTick(), last_stats = HAL_GetTick();
  capture_stats_t prev = {0};
  uint32_t prev_sent = 0;
  while (1)
  {
    ethernetif_poll(&gnetif);   /* received packets -> lwIP, link up/down */
    sys_check_timeouts();       /* lwIP timers (ARP, TCP) */
    if (camera_ok)
    {
      capture_poll();           /* finished frame -> ready buffer, restart DCMI */
      http_server_poll();       /* push the newest frame to the stream client */
    }

    /* Every 5 s: captured fps, JPEG sizes, where frames were lost, fps sent. */
    if (camera_ok && HAL_GetTick() - last_stats >= 5000)
    {
      const capture_stats_t *s = capture_stats();
      uint32_t dt = HAL_GetTick() - last_stats;
      uint32_t ok = s->frames_ok - prev.frames_ok;
      uint32_t sent = http_frames_sent() - prev_sent;
      last_stats = HAL_GetTick();
      printf("[cap] %lu.%lu fps captured, %lu.%lu fps sent | JPEG avg %lu KB, max %lu KB | "
             "bad %lu, too big %lu, skipped %lu, dcmi err %lu (dcmi 0x%lX dma 0x%lX), timeouts %lu\r\n",
             (unsigned long)(ok * 1000 / dt), (unsigned long)(ok * 10000 / dt % 10),
             (unsigned long)(sent * 1000 / dt), (unsigned long)(sent * 10000 / dt % 10),
             (unsigned long)(ok ? (uint32_t)((s->bytes_ok - prev.bytes_ok) / ok / 1024) : 0),
             (unsigned long)(s->max_size / 1024),
             (unsigned long)(s->frames_bad - prev.frames_bad),
             (unsigned long)(s->frames_too_big - prev.frames_too_big),
             (unsigned long)(s->frames_skipped - prev.frames_skipped),
             (unsigned long)(s->dcmi_errors - prev.dcmi_errors),
             (unsigned long)s->last_dcmi_error, (unsigned long)s->last_dma_error,
             (unsigned long)(s->timeouts - prev.timeouts));
      if (s->frames_too_big != prev.frames_too_big)
        printf("[cap] full buffer: first FFD8 at %ld, first FFD9 at %ld, starts %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               (long)s->big_first_ffd8, (long)s->big_first_ffd9,
               s->big_head[0], s->big_head[1], s->big_head[2], s->big_head[3],
               s->big_head[4], s->big_head[5], s->big_head[6], s->big_head[7]);
      if (s->dcmi_errors != prev.dcmi_errors)
        printf("[cap] at last error: DMA NDTR %lu (moved %lu bytes), DMA CR 0x%08lX, LISR 0x%08lX, DCMI SR 0x%lX\r\n",
               (unsigned long)s->err_ndtr, (unsigned long)((112U * 1024U / 4U - s->err_ndtr) * 4U),
               (unsigned long)s->err_dma_cr, (unsigned long)s->err_dma_isr, (unsigned long)s->err_dcmi_sr);
      prev = *s;
      prev_sent = http_frames_sent();
    }

    if (HAL_GetTick() - last_blink >= 250)
    {
      last_blink = HAL_GetTick();
      HAL_GPIO_TogglePin(LED_GREEN_PORT, LED_GREEN_PIN);
    }

    /* DHT11 once per second: temperature, humidity, checksum pass/fail. */
    if (HAL_GetTick() - last_dht >= 1000)
    {
      dht11_reading_t r;
      last_dht = HAL_GetTick();
      dht11_status_t s = dht11_read(&r);
      if (s == DHT11_OK)
        printf("[dht11] %u.%u C  %u %% RH  checksum ok", r.temperature, r.temperature_dec, r.humidity);
      else
        printf("[dht11] read failed: %s", dht11_status_text(s));
      printf("   VDD %lu mV\r\n", (unsigned long)measure_vdd_mv());
      co_sensor_update();
      {
        co_reading_t c;
        uint32_t age;
        co_sensor_last(&c, &age);
        if (c.warm)
          printf("[co] %lu mV  Rs %lu k  R0 %lu k  CO ~%lu.%lu ppm (estimate)\r\n",
                 (unsigned long)c.mv, (unsigned long)(c.rs_ohm / 1000), (unsigned long)(c.r0_ohm / 1000),
                 (unsigned long)c.ppm, (unsigned long)((uint32_t)(c.ppm * 10) % 10));
        else
          printf("[co] %lu mV  Rs %lu k  warming up (%lu s left)\r\n",
                 (unsigned long)c.mv, (unsigned long)(c.rs_ohm / 1000),
                 (unsigned long)((CO_WARMUP_MS - HAL_GetTick()) / 1000));
      }
    }
  }
}

/* Expands BOARD_IP etc. into four numbers before IP4_ADDR sees them. */
#define IP4_SET(addr, ...) IP4_ADDR(addr, __VA_ARGS__)

static void Netif_Config(void)
{
  ip4_addr_t ip, mask, gw;

  IP4_SET(&ip, BOARD_IP);
  IP4_SET(&mask, BOARD_NETMASK);
  IP4_SET(&gw, BOARD_GATEWAY);

  netif_add(&gnetif, &ip, &mask, &gw, NULL, ethernetif_init, ethernet_input);
  netif_set_default(&gnetif);
}

/*
 * 108 MHz from the 8 MHz clock the ST-LINK feeds into PH0 (HSE bypass):
 *   8 MHz / M 8 * N 216 / P 2 = 108 MHz SYSCLK
 *   AHB 108 MHz, APB1 54 MHz (/2), APB2 108 MHz (/1), flash 3 wait states.
 * Half of the F746's 216 MHz maximum, to cut the board's current: everything
 * runs from one USB cable through a 500 mA switch, and with the camera on the
 * rails sagged far enough (3.3 V -> 2.5 V) to make the DHT11 and Ethernet flaky.
 * Streaming ~1 MB/s needs only a fraction of this. APB1/APB2 keep the same
 * frequencies as before (54/108 MHz), so I2C timing, TIM2 (1 MHz), the ADC
 * and USART3 need no other change. For 216 MHz: N 432, APB1 /4, APB2 /2,
 * 7 wait states, plus HAL_PWREx_EnableOverDrive().
 */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);   /* lowest scale, fine up to 144 MHz */

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_BYPASS;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 8;
  osc.PLL.PLLN = 216;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    Error_Handler();

  clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK)
    Error_Handler();
}

static void GPIO_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* LEDs LD1/LD2/LD3, off */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_PIN | LED_BLUE_PIN | LED_RED_PIN, GPIO_PIN_RESET);
  g.Pin = LED_GREEN_PIN | LED_BLUE_PIN | LED_RED_PIN;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);

  /* Camera: powered down (PWDN high) and held in reset (RESET low) until step 2. */
  HAL_GPIO_WritePin(CAM_PWDN_PORT, CAM_PWDN_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CAM_RESET_PORT, CAM_RESET_PIN, GPIO_PIN_RESET);
  g.Pin = CAM_PWDN_PIN | CAM_RESET_PIN;
  HAL_GPIO_Init(GPIOE, &g);
}

static void USART3_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
    Error_Handler();
}

/* USART3 pins: PD8 = TX, PD9 = RX (AF7), wired to the ST-LINK virtual COM port. */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef g = {0};

  if (huart->Instance != USART3)
    return;
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &g);
}

/* Fatal error: red LED on, stop. */
void Error_Handler(void)
{
  __disable_irq();
  HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
  while (1)
  {
  }
}
