/*
 * LED mode commands over UDP (port 5005) and HTTP (/led, see http_server.c).
 * lwIP raw API: the receive callback runs from the main loop (ethernetif_poll).
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "main.h"
#include "lwip/udp.h"
#include "led_command.h"
#include "led_modes.h"
#include "ws2812.h"

static int is_status(const char *text)
{
  while (*text && isspace((unsigned char)*text))
    text++;
  for (const char *w = "STATUS"; *w; w++, text++)
    if (toupper((unsigned char)*text) != *w)
      return 0;
  return *text == '\0' || isspace((unsigned char)*text);
}

int led_command_handle(const char *text, char *reply, size_t size)
{
  led_mode_t m;
  int n;

  if (led_mode_parse(text, &m))
  {
    led_modes_command(m);
    n = snprintf(reply, size, "OK %s\n", led_mode_name(m));
  }
  else if (is_status(text))
  {
    n = snprintf(reply, size, "STATUS mode=%s ms_since_command=%lu timeout_ms=%lu frames=%lu errors=%lu\n",
                 led_mode_name(led_modes_current()), (unsigned long)led_modes_ms_since_command(),
                 (unsigned long)LED_CMD_TIMEOUT_MS, (unsigned long)ws2812_frames_sent(),
                 (unsigned long)ws2812_errors());
  }
  else
  {
    n = snprintf(reply, size, "ERR unknown command (use STRAIGHT LEFT RIGHT UNSAFE OFF TEST STATUS)\n");
  }
  if (n < 0)
    return 0;
  return n < (int)size ? n : (int)size - 1;
}

static void on_udp(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  char text[48], reply[160];
  (void)arg;

  u16_t n = pbuf_copy_partial(p, text, sizeof(text) - 1, 0);
  text[n] = '\0';
  pbuf_free(p);

  int len = led_command_handle(text, reply, sizeof(reply));
  struct pbuf *r = pbuf_alloc(PBUF_TRANSPORT, (u16_t)len, PBUF_RAM);
  if (r == NULL)
    return;
  memcpy(r->payload, reply, (size_t)len);
  udp_sendto(pcb, r, addr, port);
  pbuf_free(r);
}

void led_command_init(void)
{
  struct udp_pcb *pcb = udp_new();

  if (pcb == NULL || udp_bind(pcb, IP_ADDR_ANY, LED_UDP_PORT) != ERR_OK)
  {
    printf("[led] could not open UDP port %d\r\n", LED_UDP_PORT);
    return;
  }
  udp_recv(pcb, on_udp, NULL);
  printf("[led] commands on UDP port %d and http://192.168.10.2/led?mode=LEFT\r\n", LED_UDP_PORT);
}
