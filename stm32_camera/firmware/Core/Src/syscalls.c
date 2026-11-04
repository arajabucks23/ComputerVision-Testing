/* printf() -> USART3 (ST-LINK virtual COM port) and the Ethernet log. The other C library stubs come from nosys.specs. */
#include "main.h"
#include "log.h"

int _write(int file, char *ptr, int len)
{
  (void)file;
  log_write(ptr, len);
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}
