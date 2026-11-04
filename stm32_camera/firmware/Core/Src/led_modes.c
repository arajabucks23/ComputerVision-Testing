/*
 * Evacuation guidance patterns, drawn from the main loop at ~50 frames/s.
 *
 * Supply current (5 V strip supply, ~20 mA per colour channel at level 255, plus
 * up to ~1 mA per LED when dark, i.e. ~0.16 A for 160 LEDs):
 *   TEST ~0.18 A, NO_SIGNAL ~0.23 A, LEFT/RIGHT/STRAIGHT ~0.22 A, UNSAFE ~0.76 A peak.
 * ws2812.c caps every channel at WS2812_MAX_LEVEL whatever is drawn here.
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "main.h"
#include "ws2812.h"
#include "led_modes.h"

#define FRAME_MS        20U

#define TEST_LEDS       8U
#define TEST_LEVEL      8U

#define CHASE_SPACING   16U          /* one comet every 16 LEDs */
#define CHASE_STEP_MS   25U          /* 40 LEDs/s */
static const uint8_t comet[] = {40, 20, 10, 5, 2};   /* head first */

#define UNSAFE_MS       500U         /* 2 Hz, half on */
#define UNSAFE_LEVEL    48U

#define NOSIG_MS        2000U        /* 1 s on, 1 s off */
#define NOSIG_EVERY     4U

static led_mode_t mode = LED_MODE_TEST;
static uint32_t last_command;
static uint32_t last_frame;

static const char *const names[] = {
  [LED_MODE_OFF] = "OFF",
  [LED_MODE_TEST] = "TEST",
  [LED_MODE_STRAIGHT] = "STRAIGHT",
  [LED_MODE_LEFT] = "LEFT",
  [LED_MODE_RIGHT] = "RIGHT",
  [LED_MODE_UNSAFE] = "UNSAFE",
  [LED_MODE_NO_SIGNAL] = "NO_SIGNAL",
};

const char *led_mode_name(led_mode_t m)
{
  return (unsigned)m < sizeof(names) / sizeof(names[0]) ? names[m] : "?";
}

int led_mode_parse(const char *text, led_mode_t *out)
{
  char word[16];
  size_t n = 0;

  while (*text && isspace((unsigned char)*text))
    text++;
  while (*text && !isspace((unsigned char)*text) && n < sizeof(word) - 1)
    word[n++] = (char)toupper((unsigned char)*text++);
  word[n] = '\0';

  for (led_mode_t m = LED_MODE_OFF; m < LED_MODE_NO_SIGNAL; m++)
    if (strcmp(word, names[m]) == 0)
    {
      *out = m;
      return 1;
    }
  return 0;
}

/* Comet brightness at position p for comets moving towards higher p (0 = dark). */
static uint8_t chase_level(uint32_t p, uint32_t now)
{
  uint32_t offset = (now / CHASE_STEP_MS) % CHASE_SPACING;
  /* Distance behind the nearest head; the tail trails opposite to the motion. */
  uint32_t d = (offset + CHASE_SPACING - p % CHASE_SPACING) % CHASE_SPACING;

  return d < sizeof(comet) ? comet[d] : 0;
}

/* Comets moving towards higher LED numbers (towards_high = 1) or lower ones. */
static void draw_chase(int towards_high, uint32_t now)
{
  for (uint32_t i = 0; i < WS2812_NUM_LEDS; i++)
    ws2812_set(i, 0, chase_level(towards_high ? i : WS2812_NUM_LEDS - 1U - i, now), 0);
}

/* Both halves move towards the middle; mirrored, so it doesn't depend on LED_INDEX0_AT_LEFT. */
static void draw_converge(uint32_t now)
{
  for (uint32_t i = 0; i < WS2812_NUM_LEDS; i++)
  {
    uint32_t p = i < WS2812_NUM_LEDS / 2U ? i : WS2812_NUM_LEDS - 1U - i;
    ws2812_set(i, 0, chase_level(p, now), 0);
  }
}

void led_modes_draw(led_mode_t m, uint32_t now)
{
  ws2812_clear();
  switch (m)
  {
    case LED_MODE_OFF:
      break;

    case LED_MODE_TEST:
      for (uint32_t i = 0; i < TEST_LEDS; i++)
        ws2812_set(i, TEST_LEVEL, TEST_LEVEL, TEST_LEVEL);
      break;

    case LED_MODE_STRAIGHT:
      draw_converge(now);
      break;

    case LED_MODE_LEFT:
      draw_chase(!LED_INDEX0_AT_LEFT, now);
      break;

    case LED_MODE_RIGHT:
      draw_chase(LED_INDEX0_AT_LEFT, now);
      break;

    case LED_MODE_UNSAFE:
      if (now % UNSAFE_MS < UNSAFE_MS / 2U)
        for (uint32_t i = 0; i < WS2812_NUM_LEDS; i++)
          ws2812_set(i, UNSAFE_LEVEL, 0, 0);
      break;

    case LED_MODE_NO_SIGNAL:
      if (now % NOSIG_MS < NOSIG_MS / 2U)
        for (uint32_t i = 0; i < WS2812_NUM_LEDS; i += NOSIG_EVERY)
          ws2812_set(i, 16, 6, 0);
      break;
  }
}

void led_modes_init(void)
{
  mode = LED_MODE_TEST;
  last_command = HAL_GetTick();
  led_modes_draw(mode, last_command);
  uint32_t t = HAL_GetTick();
  while (ws2812_show() != 0 && HAL_GetTick() - t < 50U)
  {
  }
  last_frame = HAL_GetTick();
  printf("[led] TEST pattern: LEDs 0-%u dim white, %u-%u off. NO_SIGNAL after %lu s without a command\r\n",
         (unsigned)(TEST_LEDS - 1U), (unsigned)TEST_LEDS, (unsigned)(WS2812_NUM_LEDS - 1U),
         (unsigned long)(LED_CMD_TIMEOUT_MS / 1000U));
}

void led_modes_command(led_mode_t m)
{
  last_command = HAL_GetTick();
  if (m != mode)
  {
    printf("[led] mode %s -> %s\r\n", led_mode_name(mode), led_mode_name(m));
    mode = m;
  }
}

void led_modes_poll(void)
{
  uint32_t now = HAL_GetTick();

  if (mode != LED_MODE_NO_SIGNAL && now - last_command > LED_CMD_TIMEOUT_MS)
  {
    printf("[led] no command for %lu s: %s -> NO_SIGNAL\r\n",
           (unsigned long)(LED_CMD_TIMEOUT_MS / 1000U), led_mode_name(mode));
    mode = LED_MODE_NO_SIGNAL;
  }

  if (now - last_frame < FRAME_MS || ws2812_busy())
    return;
  led_modes_draw(mode, now);
  if (ws2812_show() == 0)
    last_frame = now;
}

led_mode_t led_modes_current(void)
{
  return mode;
}

uint32_t led_modes_ms_since_command(void)
{
  return HAL_GetTick() - last_command;
}
