/*
 * LED mode commands from the laptop.
 *
 *   UDP port 5005: one ASCII command per datagram, e.g. "LEFT" (case-insensitive,
 *                  trailing newline allowed). The board replies to the sender.
 *   HTTP:          GET /led?mode=LEFT   (and GET /led for the status)
 *
 * Commands: STRAIGHT, LEFT, RIGHT, UNSAFE, OFF, TEST set the mode; STATUS only reports.
 * Every mode command (even a repeat of the current mode) restarts the 5 s no-signal
 * timeout, so the CV program should resend its decision at least every 1-2 s.
 */
#ifndef LED_COMMAND_H
#define LED_COMMAND_H

#include <stddef.h>

#define LED_UDP_PORT  5005

void led_command_init(void);                                  /* opens the UDP port */
int led_command_handle(const char *text, char *reply, size_t size);   /* returns reply length */

#endif /* LED_COMMAND_H */
