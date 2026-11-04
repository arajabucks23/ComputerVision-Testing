/* DHT11 temperature/humidity sensor on PE4 (single-wire, open-drain). */
#ifndef DHT11_H
#define DHT11_H

#include <stdint.h>

typedef enum
{
  DHT11_OK = 0,
  DHT11_NO_RESPONSE,      /* sensor never pulled the line low: check DATA wire / power */
  DHT11_TIMEOUT,          /* reply stopped part-way through the 40 bits */
  DHT11_BAD_CHECKSUM,     /* all 40 bits arrived but the checksum does not match */
} dht11_status_t;

typedef struct
{
  uint8_t humidity;        /* % RH, integer part (DHT11 resolution is 1 %) */
  uint8_t humidity_dec;
  uint8_t temperature;     /* deg C, integer part */
  uint8_t temperature_dec;
} dht11_reading_t;

void dht11_init(void);
/* Blocks ~25 ms. Read at most once per second, and not in the first second after power-up. */
dht11_status_t dht11_read(dht11_reading_t *out);
const char *dht11_status_text(dht11_status_t s);

/* Last good reading and its age in ms. Returns 0 if there has never been one. */
int dht11_last(dht11_reading_t *out, uint32_t *age_ms);
void dht11_probe(void);   /* diagnostic: raw edges on DATA after a start pulse */

#endif /* DHT11_H */
