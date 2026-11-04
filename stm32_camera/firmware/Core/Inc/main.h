/*
 * Board settings for the EvacAI camera node (NUCLEO-F746ZG + OV5640).
 * Everything that would normally live in a CubeMX .ioc file is listed here.
 */
#ifndef MAIN_H
#define MAIN_H

#include "stm32f7xx_hal.h"

/* ---------- Network (static, no DHCP) ---------- */
#define BOARD_IP        192, 168, 10, 2
#define BOARD_NETMASK   255, 255, 255, 0
#define BOARD_GATEWAY   192, 168, 10, 1     /* the PC */

/* ---------- Nucleo LEDs ---------- */
#define LED_GREEN_PORT  GPIOB               /* LD1 */
#define LED_GREEN_PIN   GPIO_PIN_0
#define LED_BLUE_PORT   GPIOB               /* LD2 */
#define LED_BLUE_PIN    GPIO_PIN_7
#define LED_RED_PORT    GPIOB               /* LD3 */
#define LED_RED_PIN     GPIO_PIN_14

/* ---------- Camera control pins (OV5640 breakout) ---------- */
#define CAM_PWDN_PORT   GPIOE               /* PD: HIGH = camera powered down */
#define CAM_PWDN_PIN    GPIO_PIN_8
#define CAM_RESET_PORT  GPIOE               /* RT: LOW = camera held in reset */
#define CAM_RESET_PIN   GPIO_PIN_7

/* Camera clock. 1 = the STM32 drives XC with 16 MHz on PA8 (MCO1, from HSI); needs a
 * wire XC -> PA8 and the breakout's XCLK jumper on EXT (factory setting).
 * 0 = the breakout's own 24 MHz oscillator (XCLK jumper moved to INT, XC unconnected). */
#define CAMERA_CLOCK_FROM_MCO  1

/* 1 = run the sensor's internal clocks at half speed (~8 fps instead of ~17).
 * The camera then draws less current, which keeps the Nucleo's USB-powered 3.3 V
 * high enough for Ethernet until the camera gets its own 3.3 V supply. */
#define CAMERA_HALF_SPEED      1

/* Debug serial: USART3 on PD8 (TX) / PD9 (RX) = ST-LINK virtual COM port, 115200 8N1 */
extern UART_HandleTypeDef huart3;

void Error_Handler(void);

#endif /* MAIN_H */
