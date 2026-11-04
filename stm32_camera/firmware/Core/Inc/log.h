/* Copy of everything printf() prints, so it can be read over Ethernet too
 * (http://192.168.10.2/log) when no PC is on the ST-LINK USB. */
#ifndef LOG_H
#define LOG_H

#include <stdint.h>

void log_write(const char *data, int len);

/* Calls out() for each stored piece in order: the start of the boot log, then the
 * most recent output. Pieces are contiguous chunks of the internal buffers. */
void log_for_each(void (*out)(const char *data, uint32_t len, void *ctx), void *ctx);

#endif /* LOG_H */
