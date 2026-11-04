/* OV5640 control: power/reset pins and register access over SCCB (I2C2). */
#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#define OV5640_I2C_ADDR     (0x3C << 1)   /* 7-bit address 0x3C, HAL wants it shifted */
#define OV5640_REG_ID_HIGH  0x300A        /* reads 0x56 */
#define OV5640_REG_ID_LOW   0x300B        /* reads 0x40 */

extern I2C_HandleTypeDef hi2c2;

void camera_clock_init(void);           /* XC clock from PA8 if CAMERA_CLOCK_FROM_MCO */
void camera_bus_init(void);             /* I2C2 on PF0 (SDA) / PF1 (SCL), 100 kHz */
void camera_power_on(void);             /* PWDN low, then release RESET; ready for SCCB */
void camera_power_off(void);            /* RESET low, PWDN high */
bool camera_answers(void);              /* true if something ACKs at 0x3C */
HAL_StatusTypeDef camera_write(uint16_t reg, uint8_t value);
HAL_StatusTypeDef camera_read(uint16_t reg, uint8_t *value);

/* JPEG output (uses ST's stm32-ov5640 driver). resolution: OV5640_R640x480 etc. from ov5640.h.
 * Powers the camera on, configures it and leaves it streaming. Returns 0 on success. */
int camera_start_jpeg(uint32_t resolution);

/* Watches VSYNC (PG9) and HREF (PA4) for 'ms' milliseconds and prints frames per second
 * and HREF lines per frame. Blocks; used to check the sensor before DCMI capture exists. */
void camera_measure_sync(uint32_t ms);
void camera_probe_bus(uint32_t ms);     /* diagnostic: data bus activity vs HREF */

#endif /* CAMERA_H */
