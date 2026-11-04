/*
 * Wiring self-test for the OV5640 breakout and the DHT11.
 *
 * 1. I2C lines idle high        -> the breakout's pull-ups are powered (3V, GND)
 * 2. Camera answers at 0x3C,
 *    chip ID 0x5640             -> SDA, SCL, power and the INT clock jumper work
 * 3. RT low / PD high silence it -> the reset and power-down wires work
 * 4. Data/sync pins: the OV5640 can drive each of its output pins from a register
 *    (pad select 0x301D/0x301E, output enable 0x3017/0x3018, value 0x301A/0x301B).
 *    Each camera pin is driven high on its own while the STM32 reads all 11 pins,
 *    which finds open, swapped and shorted wires.
 * 5. DHT11 gives a reading with a valid checksum.
 */
#include <stdio.h>
#include "main.h"
#include "camera.h"
#include "dht11.h"
#include "adc.h"
#include "wiring_check.h"

typedef struct
{
  const char *cam;       /* label on the Adafruit breakout */
  const char *stm;       /* NUCLEO pin name */
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t group;         /* 0: regs 0x3017/0x301A/0x301D, 1: regs 0x3018/0x301B/0x301E */
  uint8_t bit;
} cam_signal_t;

static const cam_signal_t SIGNALS[] = {
  {"D2", "PC6",  GPIOC, GPIO_PIN_6,  1, 4},
  {"D3", "PC7",  GPIOC, GPIO_PIN_7,  1, 5},
  {"D4", "PC8",  GPIOC, GPIO_PIN_8,  1, 6},
  {"D5", "PC9",  GPIOC, GPIO_PIN_9,  1, 7},
  {"D6", "PC11", GPIOC, GPIO_PIN_11, 0, 0},
  {"D7", "PD3",  GPIOD, GPIO_PIN_3,  0, 1},
  {"D8", "PB8",  GPIOB, GPIO_PIN_8,  0, 2},
  {"D9", "PB9",  GPIOB, GPIO_PIN_9,  0, 3},
  {"PC", "PA6",  GPIOA, GPIO_PIN_6,  0, 4},   /* pixel clock */
  {"HS", "PA4",  GPIOA, GPIO_PIN_4,  0, 5},   /* HREF */
  {"VS", "PG9",  GPIOG, GPIO_PIN_9,  0, 6},   /* VSYNC */
};
#define NUM_SIGNALS (sizeof(SIGNALS) / sizeof(SIGNALS[0]))

static const uint8_t GROUP_MASK[2] = {0x7F, 0xF0};  /* VS,HS,PC,D9..D6 | D5..D2 */

static int problems;

static void report(int ok, const char *what, const char *detail)
{
  printf("[check] %-4s %-34s %s\r\n", ok ? "OK" : "FAIL", what, detail);
  if (!ok)
    problems++;
}

static void set_stm_inputs(uint32_t pull)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = pull;
  for (unsigned i = 0; i < NUM_SIGNALS; i++)
  {
    g.Pin = SIGNALS[i].pin;
    HAL_GPIO_Init(SIGNALS[i].port, &g);
  }
  HAL_Delay(1);
}

/* Bit i set = STM32 pin of SIGNALS[i] reads high. */
static uint32_t read_stm_pins(void)
{
  uint32_t v = 0;

  for (unsigned i = 0; i < NUM_SIGNALS; i++)
    if (HAL_GPIO_ReadPin(SIGNALS[i].port, SIGNALS[i].pin) == GPIO_PIN_SET)
      v |= 1U << i;
  return v;
}

/* Drive the camera's pins: bit i of 'levels' = level of SIGNALS[i]. */
static void drive_cam_pins(uint32_t levels)
{
  uint8_t value[2] = {0, 0};

  for (unsigned i = 0; i < NUM_SIGNALS; i++)
    if (levels & (1U << i))
      value[SIGNALS[i].group] |= 1U << SIGNALS[i].bit;
  camera_write(0x301A, value[0]);
  camera_write(0x301B, value[1]);
  HAL_Delay(1);
}

static void check_i2c_idle(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLDOWN;   /* only the breakout's pull-ups can make these read high */
  HAL_GPIO_Init(GPIOF, &g);
  HAL_Delay(1);
  int sda = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == GPIO_PIN_SET;
  int scl = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) == GPIO_PIN_SET;

  report(sda, "SDA -> PF0 idles high", sda ? "pull-up present" : "reads low: SDA wire, camera 3V or GND");
  report(scl, "SCL -> PF1 idles high", scl ? "pull-up present" : "reads low: SCL wire, camera 3V or GND");
}

static int check_camera_i2c(void)
{
  uint8_t hi = 0, lo = 0;

  camera_power_on();
  if (!camera_answers())
  {
    char found[64] = "nothing";
    int n = 0;
    for (uint16_t a = 0x08; a < 0x78 && n < 50; a++)
      if (HAL_I2C_IsDeviceReady(&hi2c2, a << 1, 1, 2) == HAL_OK)
        n += snprintf(found + n, sizeof(found) - n, "0x%02X ", a);
    printf("[check]      I2C2 bus scan found: %s\r\n", found);
    report(0, "camera answers at 0x3C",
           "no: check SDA/SCL not swapped, 3V/GND, clock jumper on INT");
    return 0;
  }
  report(1, "camera answers at 0x3C", "yes");

  int ok = camera_read(OV5640_REG_ID_HIGH, &hi) == HAL_OK && camera_read(OV5640_REG_ID_LOW, &lo) == HAL_OK;
  char detail[48];
  snprintf(detail, sizeof(detail), "0x%02X%02X (expect 0x5640)", hi, lo);
  report(ok && hi == 0x56 && lo == 0x40, "chip ID 0x300A/0x300B", detail);
  return 1;
}

/* ---------------------------------------------------------------------------
 * Diagnosis when the camera does not answer
 * ------------------------------------------------------------------------- */

/* Read a pin with the STM32's weak pull-up, then pull-down. The breakout pulls
 * PD down and RT up (stronger than the STM32's ~40k), so:
 *   always low  -> wired to PD,  always high -> wired to RT,  follows pull -> not connected. */
static const char *probe_control_pin(uint16_t pin)
{
  GPIO_InitTypeDef g = {0};
  int with_up, with_down;

  g.Pin = pin;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &g);
  HAL_Delay(2);
  with_up = HAL_GPIO_ReadPin(GPIOE, pin) == GPIO_PIN_SET;
  g.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &g);
  HAL_Delay(2);
  with_down = HAL_GPIO_ReadPin(GPIOE, pin) == GPIO_PIN_SET;

  /* back to output, camera off */
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &g);

  if (!with_up && !with_down)
    return "PD (breakout pull-down seen)";
  if (with_up && with_down)
    return "RT (breakout pull-up seen)";
  if (with_up && !with_down)
    return "nothing (follows STM32 pull)";
  return "? (inverted)";
}

/* Minimal software I2C: is anything ACKing 'addr7' with SDA/SCL on the given pins? */
static void sw_delay(void)
{
  uint32_t t = TIM2->CNT;   /* 1 MHz, started by dht11_init() */
  while (TIM2->CNT - t < 5)
  {
  }
}

static int sw_i2c_ack(uint16_t sda, uint16_t scl, uint8_t addr7)
{
  GPIO_InitTypeDef g = {0};
  int ack;

  HAL_GPIO_WritePin(GPIOF, sda | scl, GPIO_PIN_SET);
  g.Pin = sda | scl;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &g);
  sw_delay();

  HAL_GPIO_WritePin(GPIOF, sda, GPIO_PIN_RESET);   /* START */
  sw_delay();
  HAL_GPIO_WritePin(GPIOF, scl, GPIO_PIN_RESET);
  uint8_t byte = (uint8_t)(addr7 << 1);            /* write address */
  for (int i = 7; i >= 0; i--)
  {
    HAL_GPIO_WritePin(GPIOF, sda, (byte >> i) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    sw_delay();
    HAL_GPIO_WritePin(GPIOF, scl, GPIO_PIN_SET);
    sw_delay();
    HAL_GPIO_WritePin(GPIOF, scl, GPIO_PIN_RESET);
  }
  HAL_GPIO_WritePin(GPIOF, sda, GPIO_PIN_SET);     /* release SDA for ACK */
  sw_delay();
  HAL_GPIO_WritePin(GPIOF, scl, GPIO_PIN_SET);
  sw_delay();
  ack = HAL_GPIO_ReadPin(GPIOF, sda) == GPIO_PIN_RESET;
  HAL_GPIO_WritePin(GPIOF, scl, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOF, sda, GPIO_PIN_RESET);   /* STOP */
  sw_delay();
  HAL_GPIO_WritePin(GPIOF, scl, GPIO_PIN_SET);
  sw_delay();
  HAL_GPIO_WritePin(GPIOF, sda, GPIO_PIN_SET);
  sw_delay();
  return ack;
}

static void diagnose_no_answer(void)
{
  printf("[diag] PE7 (meant for RT) is connected to: %s\r\n", probe_control_pin(CAM_RESET_PIN));
  printf("[diag] PE8 (meant for PD) is connected to: %s\r\n", probe_control_pin(CAM_PWDN_PIN));

  HAL_I2C_DeInit(&hi2c2);   /* use PF0/PF1 as plain pins for the software I2C probe */
  for (int swapped = 0; swapped < 2; swapped++)
  {
    uint16_t sda = swapped ? GPIO_PIN_1 : GPIO_PIN_0;
    uint16_t scl = swapped ? GPIO_PIN_0 : GPIO_PIN_1;
    for (int combo = 0; combo < 4; combo++)
    {
      int pe7 = combo & 1, pe8 = (combo >> 1) & 1;
      HAL_GPIO_WritePin(CAM_RESET_PORT, CAM_RESET_PIN, GPIO_PIN_RESET);   /* restart the camera */
      HAL_GPIO_WritePin(CAM_PWDN_PORT, CAM_PWDN_PIN, GPIO_PIN_SET);
      HAL_Delay(5);
      HAL_GPIO_WritePin(CAM_PWDN_PORT, CAM_PWDN_PIN, pe8 ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_Delay(5);
      HAL_GPIO_WritePin(CAM_RESET_PORT, CAM_RESET_PIN, pe7 ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_Delay(30);
      int ack = sw_i2c_ack(sda, scl, 0x3C);
      printf("[diag] SDA=%s SCL=%s  PE7=%d PE8=%d  -> camera %s\r\n",
             swapped ? "PF1" : "PF0", swapped ? "PF0" : "PF1", pe7, pe8, ack ? "ANSWERS" : "silent");
    }
  }
  camera_power_off();
  camera_bus_init();
}

static void check_control_pins(void)
{
  /* RT: hold reset (PWDN stays low). The camera must stop answering. */
  HAL_GPIO_WritePin(CAM_RESET_PORT, CAM_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  int silent = !camera_answers();
  report(silent, "RT -> PE7 (reset, active low)", silent ? "camera goes silent in reset" : "camera still answers: RT wire?");
  camera_power_on();

  /* PD: power down (RESET stays high). */
  HAL_GPIO_WritePin(CAM_PWDN_PORT, CAM_PWDN_PIN, GPIO_PIN_SET);
  HAL_Delay(5);
  silent = !camera_answers();
  report(silent, "PD -> PE8 (power-down, active high)", silent ? "camera goes silent when powered down" : "camera still answers: PD wire?");
  camera_power_on();
}

static void check_data_pins(void)
{
  uint32_t all = (1U << NUM_SIGNALS) - 1;
  uint32_t stuck_high, stuck_low, seen[NUM_SIGNALS];

  /* Hand the camera's pins to register control and enable their outputs. */
  camera_write(0x301D, GROUP_MASK[0]);
  camera_write(0x301E, GROUP_MASK[1]);
  camera_write(0x3017, GROUP_MASK[0]);
  camera_write(0x3018, GROUP_MASK[1]);

  /* All low, STM32 pull-ups on: a pin that still reads high is not being driven. */
  set_stm_inputs(GPIO_PULLUP);
  drive_cam_pins(0);
  stuck_high = read_stm_pins();

  /* All high, STM32 pull-downs on: a pin that still reads low is not being driven. */
  set_stm_inputs(GPIO_PULLDOWN);
  drive_cam_pins(all);
  stuck_low = ~read_stm_pins() & all;

  /* One camera pin high at a time. */
  for (unsigned i = 0; i < NUM_SIGNALS; i++)
  {
    drive_cam_pins(1U << i);
    seen[i] = read_stm_pins();
  }

  /* Raw values, to tell a wiring fault from a register problem. */
  {
    static const uint16_t regs[] = {0x3017, 0x3018, 0x301A, 0x301B, 0x301D, 0x301E};
    drive_cam_pins(all);
    printf("[diag] pad regs with all pins driven high:");
    for (unsigned k = 0; k < sizeof(regs) / sizeof(regs[0]); k++)
    {
      uint8_t v = 0;
      HAL_StatusTypeDef st = camera_read(regs[k], &v);
      printf(" %04X=%02X%s", regs[k], v, st == HAL_OK ? "" : "(err)");
    }
    printf("\r\n[diag] bit order D2 D3 D4 D5 D6 D7 D8 D9 PC HS VS = bits 0..10\r\n");
    printf("[diag] all-low+pullup reads high: 0x%03lX   all-high+pulldown reads low: 0x%03lX\r\n",
           (unsigned long)stuck_high, (unsigned long)stuck_low);
    printf("[diag] walking one:");
    for (unsigned i = 0; i < NUM_SIGNALS; i++)
      printf(" %s=0x%03lX", SIGNALS[i].cam, (unsigned long)seen[i]);
    printf("\r\n");
  }

  for (unsigned i = 0; i < NUM_SIGNALS; i++)
  {
    char what[40], detail[80];
    uint32_t others = seen[i] & ~(1U << i);
    int n = 0;

    snprintf(what, sizeof(what), "%s -> %s", SIGNALS[i].cam, SIGNALS[i].stm);
    if (((stuck_high | stuck_low) & (1U << i)) && !(seen[i] & (1U << i)) && others)
    {
      n = snprintf(detail, sizeof(detail), "no signal on %s; camera %s shows up on", SIGNALS[i].stm, SIGNALS[i].cam);
      for (unsigned j = 0; j < NUM_SIGNALS; j++)
        if (others & (1U << j))
          n += snprintf(detail + n, sizeof(detail) - n, " %s", SIGNALS[j].stm);
      report(0, what, detail);
    }
    else if ((stuck_high | stuck_low) & (1U << i))
    {
      snprintf(detail, sizeof(detail), "no signal on %s: wire loose or on the wrong pin", SIGNALS[i].stm);
      report(0, what, detail);
    }
    else if (others)
    {
      n = snprintf(detail, sizeof(detail), "shorted to");
      for (unsigned j = 0; j < NUM_SIGNALS; j++)
        if (others & (1U << j))
          n += snprintf(detail + n, sizeof(detail) - n, " %s", SIGNALS[j].stm);
      report(0, what, detail);
    }
    else
    {
      report(1, what, "connected");
    }
  }

  /* Give the pins back to the sensor and turn the outputs off again. */
  camera_write(0x3017, 0x00);
  camera_write(0x3018, 0x00);
  camera_write(0x301D, 0x00);
  camera_write(0x301E, 0x00);
  set_stm_inputs(GPIO_NOPULL);
}

/* CO sensor on PA3 (analog). Charge the pin high, read; discharge it, read. A driven
 * pin (sensor output / divider) gives the same voltage both times; an unconnected pin
 * keeps the charge it was given, so the two readings differ a lot. */
static void check_co_sensor(void)
{
  GPIO_InitTypeDef g = {0};
  uint32_t after_high, after_low;
  char detail[80];

  __HAL_RCC_GPIOA_CLK_ENABLE();
  g.Pin = GPIO_PIN_3;
  g.Speed = GPIO_SPEED_FREQ_LOW;

  for (int pass = 0; pass < 2; pass++)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, pass == 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    g.Mode = GPIO_MODE_OUTPUT_PP;       /* precharge for 50 us through the pin's output */
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
    for (volatile int i = 0; i < 2000; i++)
    {
    }
    g.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &g);
    if (pass == 0)
      after_high = adc_pin_mv(ADC_CH_PA3);
    else
      after_low = adc_pin_mv(ADC_CH_PA3);
  }

  uint32_t diff = after_high > after_low ? after_high - after_low : after_low - after_high;
  int driven = diff < 150;
  snprintf(detail, sizeof(detail), "%lu mV (precharged high %lu / low %lu)%s",
           (unsigned long)((after_high + after_low) / 2), (unsigned long)after_high,
           (unsigned long)after_low, driven ? "" : ": looks unconnected");
  report(driven, "CO sensor out -> PA3 (analog)", detail);
}

static void check_dht11(void)
{
  dht11_reading_t r;
  char detail[64];

  HAL_Delay(1100);               /* DHT11: at most one read per second */
  dht11_status_t s = dht11_read(&r);
  if (s == DHT11_OK)
    snprintf(detail, sizeof(detail), "%u.%u C, %u %% RH, checksum ok",
             r.temperature, r.temperature_dec, r.humidity);
  else if (s == DHT11_NO_RESPONSE)
    snprintf(detail, sizeof(detail), "no response: DATA -> PE4, VCC, GND");
  else
    snprintf(detail, sizeof(detail), "%s: reply garbled (pull-up? long wire?)", dht11_status_text(s));
  report(s == DHT11_OK, "DHT11 DATA -> PE4", detail);
  if (s != DHT11_OK)
  {
    HAL_Delay(1100);
    dht11_probe();
  }
}

int wiring_check_run(void)
{
  problems = 0;
  printf("[check] ---- wiring self-test ----\r\n");

  check_i2c_idle();
  camera_clock_init();
  printf("[check]      camera clock: %s\r\n", CAMERA_CLOCK_FROM_MCO
         ? "16 MHz from STM32 PA8 -> XC (jumper on EXT)" : "breakout's 24 MHz oscillator (jumper on INT)");
  camera_bus_init();
  if (check_camera_i2c())
  {
    check_control_pins();
    check_data_pins();
  }
  else
  {
    printf("[check]      skipping RT/PD and data-pin tests (camera not answering)\r\n");
    diagnose_no_answer();
  }
  camera_power_off();

  check_co_sensor();
  check_dht11();

  if (problems == 0)
    printf("[check] ---- RESULT: all connections OK ----\r\n");
  else
    printf("[check] ---- RESULT: %d problem(s), see FAIL lines ----\r\n", problems);
  return problems;
}
