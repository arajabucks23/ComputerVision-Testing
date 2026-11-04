/*
 * OV5640 JPEG capture with DCMI + DMA.
 *
 * DCMI in JPEG mode: HSYNC (HREF) marks valid bytes, VSYNC marks the frame. The
 * frame length varies, so each capture is a "snapshot" into a buffer that is larger
 * than any frame; at the end of frame the DMA counter tells how much arrived.
 *
 * Two 112 KB buffers in SRAM1 (section .frame_buffers):
 *   - one is being filled by the DMA,
 *   - the other holds the newest complete frame for the HTTP sender.
 * When a frame finishes and the sender is not using the other buffer, they swap.
 * If the sender is still busy, the new frame is skipped (counted in frames_skipped)
 * and the next frame goes into the same buffer.
 *
 * DMA2 Stream 1 Channel 1 (the DCMI request), word transfers, direct mode (no FIFO),
 * so nothing is left behind in a DMA FIFO at the end of a frame.
 */
#include <string.h>
#include "main.h"
#include "capture.h"

#define FRAME_BUF_SIZE   (112U * 1024U)
#define FRAME_TIMEOUT_MS 1000U

static uint8_t frame_buf[2][FRAME_BUF_SIZE] __attribute__((section(".frame_buffers"), aligned(32)));

DCMI_HandleTypeDef hdcmi;
static DMA_HandleTypeDef hdma_dcmi;

static volatile uint8_t frame_done, frame_error;
static int cap_idx;                  /* buffer the DMA is filling */
static int ready_idx = -1;           /* buffer with the newest complete frame */
static uint32_t ready_off, ready_len, ready_seq;
static int ready_locked;
static uint32_t capture_started;
static capture_stats_t stats;

static void start_capture(void)
{
  frame_done = 0;
  frame_error = 0;
  capture_started = HAL_GetTick();
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)frame_buf[cap_idx], FRAME_BUF_SIZE / 4U);
  /* The HAL only enables the end-of-frame interrupt once the DMA has filled the whole
   * buffer (it expects fixed-size raw frames). A JPEG frame is shorter, so enable it
   * here, as ST's JPEG examples do. The HAL also turns overrun/error interrupts off
   * after each snapshot, so re-enable those; line and VSYNC interrupts are not needed. */
  __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_LINE | DCMI_IT_VSYNC);
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME | DCMI_IT_OVR | DCMI_IT_ERR);
}

void capture_init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();
  hdma_dcmi.Instance = DMA2_Stream1;
  hdma_dcmi.Init.Channel = DMA_CHANNEL_1;
  hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dcmi.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_dcmi.Init.Mode = DMA_NORMAL;
  hdma_dcmi.Init.Priority = DMA_PRIORITY_HIGH;
  /* FIFO on, as in ST's DCMI examples. Stopping the stream at the end of a frame
   * flushes what is left in the FIFO to memory, so no bytes are lost. */
  hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_dcmi.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
    Error_Handler();

  /* DCMI "polarity" = the level while data is NOT valid. Measured on this board with
   * ST's settings (0x4740 = 0x22): JPEG bytes arrive while HREF is LOW (HREF is high
   * when idle) and VSYNC pulses high between frames. Sample on the PCLK rising edge. */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_ENABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  __HAL_LINKDMA(&hdcmi, DMA_Handle, hdma_dcmi);
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
    Error_Handler();

  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  HAL_NVIC_SetPriority(DCMI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);

  cap_idx = 0;
  start_capture();
}

/* DCMI pins, all AF13: D0 PC6, D1 PC7, D2 PC8, D3 PC9, D4 PC11, D5 PD3, D6 PB8, D7 PB9,
 * PIXCLK PA6, HSYNC PA4, VSYNC PG9. */
void HAL_DCMI_MspInit(DCMI_HandleTypeDef *h)
{
  GPIO_InitTypeDef g = {0};
  (void)h;

  __HAL_RCC_DCMI_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF13_DCMI;

  g.Pin = GPIO_PIN_4 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &g);
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11;
  HAL_GPIO_Init(GPIOC, &g);
  g.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOD, &g);
  g.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOG, &g);
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *h)
{
  (void)h;
  frame_done = 1;
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *h)
{
  stats.last_dcmi_error = h->ErrorCode;
  stats.last_dma_error = h->DMA_Handle->ErrorCode;
  stats.err_ndtr = DMA2_Stream1->NDTR;
  stats.err_dma_cr = DMA2_Stream1->CR;
  stats.err_dma_isr = DMA2->LISR;
  stats.err_dcmi_sr = DCMI->SR;
  h->ErrorCode = HAL_DCMI_ERROR_NONE;   /* the HAL only ever ORs codes in */
  /* A DMA FIFO-error flag on its own is not fatal (the HAL reports it but keeps going). */
  if (h->ErrorCode == HAL_DCMI_ERROR_DMA && h->DMA_Handle->ErrorCode == HAL_DMA_ERROR_FE)
    return;
  frame_error = 1;
}

/* Finds the JPEG inside the captured bytes: FFD8 near the start, last FFD9. */
static int find_jpeg(const uint8_t *p, uint32_t n, uint32_t *off, uint32_t *len)
{
  uint32_t start = 0, end;

  while (start + 1 < n && start < 16 && !(p[start] == 0xFF && p[start + 1] == 0xD8))
    start++;
  if (start + 1 >= n || start >= 16)
    return 0;
  for (end = n; end >= start + 4; end--)
    if (p[end - 2] == 0xFF && p[end - 1] == 0xD9)
    {
      *off = start;
      *len = end - start;
      return 1;
    }
  return 0;
}

/* The buffer filled up before the frame ended: where are the JPEG markers? (diagnostic) */
static void analyse_full_buffer(void)
{
  const uint8_t *b = frame_buf[cap_idx];

  stats.big_first_ffd8 = stats.big_first_ffd9 = 0xFFFFFFFF;
  for (uint32_t i = 0; i + 1 < FRAME_BUF_SIZE; i++)
  {
    if (stats.big_first_ffd8 == 0xFFFFFFFF && b[i] == 0xFF && b[i + 1] == 0xD8)
      stats.big_first_ffd8 = i;
    if (b[i] == 0xFF && b[i + 1] == 0xD9)
    {
      stats.big_first_ffd9 = i;
      break;
    }
  }
  memcpy(stats.big_head, b, sizeof(stats.big_head));
  stats.frames_too_big++;
}

void capture_poll(void)
{
  if (frame_error)
  {
    if (stats.err_ndtr == 0)     /* DMA had filled the whole buffer */
      analyse_full_buffer();
    stats.dcmi_errors++;
    HAL_DCMI_Stop(&hdcmi);
    start_capture();
    return;
  }
  if (!frame_done)
  {
    if (HAL_GetTick() - capture_started > FRAME_TIMEOUT_MS)
    {
      stats.timeouts++;
      HAL_DCMI_Stop(&hdcmi);
      start_capture();
    }
    return;
  }

  /* Frame finished: bytes received = buffer size - what the DMA still had to do. */
  uint32_t left_words = __HAL_DMA_GET_COUNTER(hdcmi.DMA_Handle);
  HAL_DCMI_Stop(&hdcmi);
  uint32_t n = FRAME_BUF_SIZE - left_words * 4U;
  uint32_t off, len;

  if (left_words == 0)
  {
    analyse_full_buffer();
  }
  else if (!find_jpeg(frame_buf[cap_idx], n, &off, &len))
  {
    stats.frames_bad++;
  }
  else
  {
    stats.frames_ok++;
    stats.bytes_ok += len;
    stats.last_size = len;
    if (len > stats.max_size)
      stats.max_size = len;

    if (!ready_locked)
    {
      /* Publish this buffer, capture into the other one next. */
      ready_idx = cap_idx;
      ready_off = off;
      ready_len = len;
      ready_seq++;
      cap_idx ^= 1;
    }
    else
    {
      stats.frames_skipped++;
    }
  }
  start_capture();
}

int capture_take(uint32_t after_seq, const uint8_t **jpeg, uint32_t *len, uint32_t *seq)
{
  if (ready_idx < 0 || ready_seq == after_seq || ready_locked)
    return 0;
  ready_locked = 1;
  *jpeg = frame_buf[ready_idx] + ready_off;
  *len = ready_len;
  *seq = ready_seq;
  return 1;
}

void capture_release(void)
{
  ready_locked = 0;
}

const capture_stats_t *capture_stats(void)
{
  return &stats;
}
