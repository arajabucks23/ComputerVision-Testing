/*
 * WS2812B LED strip on PE9 (Arduino D6) = TIM1_CH1, through an SN74AHCT125N
 * (3.3 V -> 5 V) and a 330 ohm resistor to the strip's DIN.
 */
#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>

#define WS2812_NUM_LEDS   160U   /* BTF-LIGHTING WS2812B strip */

/* Hard cap on every colour channel (0-255), whatever the caller asks for.
 * Full white is ~60 mA per LED (9.6 A for 160 LEDs); 64 keeps the worst case
 * (all LEDs white) near 2.4 A. The modes in led_modes.c stay well below that. */
#define WS2812_MAX_LEVEL  64U

void ws2812_init(void);                 /* TIM1 + DMA set-up, PE9 driven low, all LEDs cleared */
void ws2812_clear(void);                /* framebuffer only: every LED off */
void ws2812_set(uint32_t i, uint8_t r, uint8_t g, uint8_t b);
int  ws2812_show(void);                 /* send the framebuffer; 0 = started, -1 = busy (try again) */
int  ws2812_busy(void);
uint32_t ws2812_frames_sent(void);
uint32_t ws2812_errors(void);
void ws2812_print_timing(void);         /* timer clock and bit timing, for checking with a scope */

#endif /* WS2812_H */
