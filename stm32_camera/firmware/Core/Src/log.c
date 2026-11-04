/*
 * Log buffer: the first 2 KB of output (boot report, wiring check) are kept for
 * good; after that a 6 KB ring keeps the most recent lines.
 */
#include "log.h"

#define BOOT_SIZE 2048U
#define RING_SIZE 6144U

static char boot[BOOT_SIZE];
static uint32_t boot_len;
static char ring[RING_SIZE];
static uint32_t ring_total;     /* bytes ever written to the ring */

void log_write(const char *data, int len)
{
  for (int i = 0; i < len; i++)
  {
    if (boot_len < BOOT_SIZE)
      boot[boot_len++] = data[i];
    else
      ring[ring_total++ % RING_SIZE] = data[i];
  }
}

void log_for_each(void (*out)(const char *data, uint32_t len, void *ctx), void *ctx)
{
  out(boot, boot_len, ctx);
  if (ring_total == 0)
    return;
  if (ring_total > RING_SIZE)
  {
    static const char gap[] = "\r\n[log] ... older lines dropped ...\r\n";
    uint32_t start = ring_total % RING_SIZE;
    out(gap, sizeof(gap) - 1, ctx);
    out(ring + start, RING_SIZE - start, ctx);
    out(ring, start, ctx);
  }
  else
  {
    out(ring, ring_total, ctx);
  }
}
