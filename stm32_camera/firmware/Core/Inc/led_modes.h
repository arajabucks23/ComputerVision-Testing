/*
 * Evacuation guidance patterns on the WS2812B strip.
 *
 *   STRAIGHT   green comets from both ends moving towards the middle
 *   LEFT       green comets travelling right-to-left
 *   RIGHT      green comets travelling left-to-right
 *   UNSAFE     all LEDs flash red, 2 Hz
 *   TEST       only LEDs 0-7, dim white; all others off (shown at boot)
 *   OFF        all off
 *   NO_SIGNAL  every 4th LED blinks dim amber, 1 s on / 1 s off. Not a command:
 *              shown when no valid command has arrived for LED_CMD_TIMEOUT_MS.
 */
#ifndef LED_MODES_H
#define LED_MODES_H

#include <stdint.h>

/* 1 = LED 0 (the DIN end, where the wire from PE9 enters) is the LEFT end of the
 * strip as seen by people following it. Set to 0 if the strip is mounted the other way. */
#define LED_INDEX0_AT_LEFT   1

#define LED_CMD_TIMEOUT_MS   5000U

typedef enum
{
  LED_MODE_OFF,
  LED_MODE_TEST,
  LED_MODE_STRAIGHT,
  LED_MODE_LEFT,
  LED_MODE_RIGHT,
  LED_MODE_UNSAFE,
  LED_MODE_NO_SIGNAL,
} led_mode_t;

void led_modes_init(void);                       /* TEST pattern now; NO_SIGNAL after the timeout */
void led_modes_poll(void);                       /* main loop: failsafe check, ~50 frames/s */
void led_modes_command(led_mode_t mode);         /* a valid command arrived: set mode, restart timeout */
led_mode_t led_modes_current(void);
uint32_t led_modes_ms_since_command(void);
void led_modes_draw(led_mode_t mode, uint32_t now_ms);   /* fill the framebuffer for one frame */

const char *led_mode_name(led_mode_t mode);
int led_mode_parse(const char *text, led_mode_t *out);   /* 1 = a commandable mode name */

#endif /* LED_MODES_H */
