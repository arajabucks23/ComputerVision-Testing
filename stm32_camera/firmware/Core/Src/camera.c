/*
 * OV5640 power/reset sequencing and register access.
 *
 * SCCB is the camera's I2C dialect: 7-bit address 0x3C, 16-bit register
 * addresses, 8-bit values. The pull-ups are on the Adafruit breakout.
 */
#include <stdio.h>
#include "camera.h"
#include "ov5640.h"

I2C_HandleTypeDef hi2c2;
static OV5640_Object_t ov5640;

void camera_clock_init(void)
{
#if CAMERA_CLOCK_FROM_MCO
  /* 16 MHz internal RC oscillator (HSI) out on PA8 = MCO1 (AF0). The OV5640 accepts 6-27 MHz. */
  __HAL_RCC_HSI_ENABLE();
  while (!__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY))
  {
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
#endif
}

void camera_bus_init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;   /* 100 kHz with I2C2 clocked from APB1 = 54 MHz */
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    Error_Handler();
}

/* I2C2 pins: PF0 = SDA, PF1 = SCL (AF4, open-drain). */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef g = {0};

  if (hi2c->Instance != I2C2)
    return;
  __HAL_RCC_GPIOF_CLK_ENABLE();
  g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  g.Mode = GPIO_MODE_AF_OD;
  g.Pull = GPIO_NOPULL;             /* breakout has the pull-ups */
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &g);
  __HAL_RCC_I2C2_CLK_ENABLE();
}

/* OV5640 datasheet power-up: PWDN low, wait >= 1 ms, RESET high, wait >= 20 ms before SCCB. */
void camera_power_on(void)
{
  HAL_GPIO_WritePin(CAM_RESET_PORT, CAM_RESET_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAM_PWDN_PORT, CAM_PWDN_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(CAM_RESET_PORT, CAM_RESET_PIN, GPIO_PIN_SET);
  HAL_Delay(30);
}

void camera_power_off(void)
{
  HAL_GPIO_WritePin(CAM_RESET_PORT, CAM_RESET_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAM_PWDN_PORT, CAM_PWDN_PIN, GPIO_PIN_SET);
}

bool camera_answers(void)
{
  return HAL_I2C_IsDeviceReady(&hi2c2, OV5640_I2C_ADDR, 2, 10) == HAL_OK;
}

HAL_StatusTypeDef camera_write(uint16_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c2, OV5640_I2C_ADDR, reg, I2C_MEMADD_SIZE_16BIT, &value, 1, 50);
}

HAL_StatusTypeDef camera_read(uint16_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(&hi2c2, OV5640_I2C_ADDR, reg, I2C_MEMADD_SIZE_16BIT, value, 1, 50);
}

/* ---------------------------------------------------------------------------
 * Bus functions for ST's driver (st_ov5640/ov5640.c)
 * ------------------------------------------------------------------------- */
static int32_t io_init(void)   { return OV5640_OK; }   /* I2C2 is set up by camera_bus_init() */
static int32_t io_deinit(void) { return OV5640_OK; }
static int32_t io_gettick(void) { return (int32_t)HAL_GetTick(); }

static int32_t io_write(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
  return HAL_I2C_Mem_Write(&hi2c2, addr, reg, I2C_MEMADD_SIZE_16BIT, data, len, 100) == HAL_OK
         ? OV5640_OK : OV5640_ERROR;
}

static int32_t io_read(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c2, addr, reg, I2C_MEMADD_SIZE_16BIT, data, len, 100) == HAL_OK
         ? OV5640_OK : OV5640_ERROR;
}

/*
 * ST's register tables assume a 24 MHz XCLK. Its PLL pre-divider (0x3037 bits 3:0)
 * is 3 (or 6), giving 8 MHz (or 4 MHz) into the sensor's PLL. With the 16 MHz clock
 * from PA8, a pre-divider of 2 (or 4) gives the same PLL input, so every internal
 * clock, the frame rate and the JPEG timing stay as ST designed them.
 */
static int adjust_pll_for_xclk(void)
{
#if CAMERA_CLOCK_FROM_MCO
  uint8_t v;

  if (camera_read(0x3037, &v) != HAL_OK)
    return -1;
  switch (v & 0x0F)
  {
    case 3: v = (uint8_t)((v & 0xF0) | 2); break;   /* 24/3 = 16/2 = 8 MHz */
    case 6: v = (uint8_t)((v & 0xF0) | 4); break;   /* 24/6 = 16/4 = 4 MHz */
    default:
      printf("[cam] PLL pre-divider %u has no 16 MHz equivalent\r\n", v & 0x0F);
      return -1;
  }
  return camera_write(0x3037, v) == HAL_OK ? 0 : -1;
#else
  return 0;
#endif
}

int camera_start_jpeg(uint32_t resolution)
{
  OV5640_IO_t io = {0};
  uint32_t id = 0;
  uint8_t pll[4];

  camera_power_on();

  io.Init = io_init;
  io.DeInit = io_deinit;
  io.Address = OV5640_I2C_ADDR;
  io.WriteReg = io_write;
  io.ReadReg = io_read;
  io.GetTick = io_gettick;
  ov5640.IsInitialized = 0;
  if (OV5640_RegisterBusIO(&ov5640, &io) != OV5640_OK || OV5640_ReadID(&ov5640, &id) != OV5640_OK)
  {
    printf("[cam] no answer on I2C\r\n");
    return -1;
  }

  /* Soft reset + ST's common settings + resolution + JPEG format; ends with streaming on. */
  if (OV5640_Init(&ov5640, resolution, OV5640_JPEG) != OV5640_OK)
  {
    printf("[cam] OV5640_Init failed\r\n");
    return -1;
  }
  if (adjust_pll_for_xclk() != 0)
    return -1;
#if CAMERA_HALF_SPEED
  /* 0x3035 bits 7:4 = system clock divider. ST sets 4 (0x41); 8 halves every
   * sensor clock, so frame rate and power both drop. */
  {
    uint8_t v;
    if (camera_read(0x3035, &v) == HAL_OK)
      camera_write(0x3035, (uint8_t)((v & 0x0F) | ((uint8_t)(((v >> 4) * 2) & 0x0F) << 4)));
  }
#endif

  for (int i = 0; i < 4; i++)
    camera_read(0x3034 + i, &pll[i]);
  printf("[cam] id 0x%04lX, JPEG mode, PLL 3034..3037 = %02X %02X %02X %02X\r\n",
         (unsigned long)id, pll[0], pll[1], pll[2], pll[3]);

  /* JPEG output settings, for the record. */
  static const uint16_t regs[] = {0x4300, 0x4407, 0x4713, 0x4740, 0x3821, 0x4602, 0x4603,
                                  0x4604, 0x4605, 0x460C, 0x3824, 0x3808, 0x3809, 0x380A, 0x380B};
  printf("[cam] regs:");
  for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++)
  {
    uint8_t v = 0;
    camera_read(regs[i], &v);
    printf(" %04X=%02X", regs[i], v);
  }
  printf("\r\n");
  return 0;
}

void camera_measure_sync(uint32_t ms)
{
  GPIO_InitTypeDef g = {0};
  uint32_t frames = 0, lines = 0;
  uint32_t last_vs, last_hs, start;

  /* VSYNC (PG9) and HREF (PA4) as plain inputs for now; DCMI takes them over in step 4. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  g.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOG, &g);
  g.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &g);

  last_vs = GPIOG->IDR & GPIO_PIN_9;
  last_hs = GPIOA->IDR & GPIO_PIN_4;
  start = HAL_GetTick();
  while (HAL_GetTick() - start < ms)
  {
    uint32_t vs = GPIOG->IDR & GPIO_PIN_9;
    uint32_t hs = GPIOA->IDR & GPIO_PIN_4;
    if (vs && !last_vs)
      frames++;
    if (hs && !last_hs)
      lines++;
    last_vs = vs;
    last_hs = hs;
  }

  printf("[cam] %lu ms: %lu VSYNC pulses = %lu.%lu fps, %lu HREF lines/frame\r\n",
         (unsigned long)ms, (unsigned long)frames,
         (unsigned long)(frames * 1000 / ms), (unsigned long)(frames * 10000 / ms % 10),
         (unsigned long)(frames ? lines / frames : 0));
}

/* Data byte on the camera bus: D0..D3 = PC6..PC9, D4 = PC11, D5 = PD3, D6 = PB8, D7 = PB9. */
static uint8_t read_bus(void)
{
  uint32_t c = GPIOC->IDR, d = GPIOD->IDR, b = GPIOB->IDR;
  return (uint8_t)(((c >> 6) & 0x0F) | (((c >> 11) & 1) << 4) | (((d >> 3) & 1) << 5) | (((b >> 8) & 3) << 6));
}

/* Diagnostic: sample the data bus for 'ms' and report, separately for HREF high and
 * HREF low, how many samples there were, how often the byte changed, and the values seen. */
void camera_probe_bus(uint32_t ms)
{
  GPIO_InitTypeDef g = {0};
  uint32_t n[2] = {0}, changes[2] = {0}, vsync_hi = 0;
  uint8_t last[2] = {0}, first[2][6];
  uint32_t nfirst[2] = {0};
  uint32_t start;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11;
  HAL_GPIO_Init(GPIOC, &g);
  g.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOD, &g);
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &g);
  g.Pin = GPIO_PIN_4 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOG, &g);

  start = HAL_GetTick();
  while (HAL_GetTick() - start < ms)
  {
    int h = (GPIOA->IDR & GPIO_PIN_4) != 0;
    uint8_t v = read_bus();
    if (GPIOG->IDR & GPIO_PIN_9)
      vsync_hi++;
    if (n[h] && v != last[h])
    {
      changes[h]++;
      if (nfirst[h] < 6)
        first[h][nfirst[h]++] = v;
    }
    last[h] = v;
    n[h]++;
  }
  for (int h = 1; h >= 0; h--)
  {
    printf("[probe] HREF %s: %lu samples, byte changed %lu times, e.g.", h ? "high" : "low ",
           (unsigned long)n[h], (unsigned long)changes[h]);
    for (uint32_t i = 0; i < nfirst[h]; i++)
      printf(" %02X", first[h][i]);
    printf(" (last %02X)\r\n", last[h]);
  }
  printf("[probe] VSYNC high in %lu%% of samples\r\n",
         (unsigned long)(n[0] + n[1] ? vsync_hi * 100 / (n[0] + n[1]) : 0));
}
