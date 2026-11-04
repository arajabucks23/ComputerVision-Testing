/* JPEG frame capture: OV5640 -> DCMI -> DMA -> two frame buffers in SRAM1. */
#ifndef CAPTURE_H
#define CAPTURE_H

#include <stdint.h>

typedef struct
{
  uint32_t frames_ok;       /* complete JPEGs (FFD8 ... FFD9) */
  uint32_t frames_bad;      /* no FFD8 at the start or no FFD9 at the end */
  uint32_t frames_too_big;  /* filled the whole buffer */
  uint32_t frames_skipped;  /* good frame, but the sender still had the other buffer */
  uint32_t dcmi_errors;     /* DCMI overrun / sync errors */
  uint32_t timeouts;        /* no frame for 1 s: capture restarted */
  uint32_t last_dcmi_error; /* HAL_DCMI_ERROR_* of the last error (OVR=0x01, SYNC=0x02, DMA=0x40) */
  uint32_t last_dma_error;  /* HAL_DMA_ERROR_* of the last error (TE=0x01, FE=0x02, DME=0x04) */
  uint32_t err_ndtr, err_dma_cr, err_dma_isr, err_dcmi_sr;   /* registers at the last error */
  uint32_t big_first_ffd8, big_first_ffd9;                    /* marker offsets in an over-full buffer */
  uint8_t big_head[16];                                       /* its first bytes */
  uint32_t last_size;
  uint32_t max_size;
  uint64_t bytes_ok;
} capture_stats_t;

void capture_init(void);      /* DCMI + DMA set-up; the camera must already be streaming JPEG */
void capture_poll(void);      /* call from the main loop: handles a finished frame, restarts capture */

/* Newest complete JPEG. Returns 1 and locks it until capture_release() (the
 * capture then never overwrites it), or 0 if there is no frame newer than 'after_seq'. */
int capture_take(uint32_t after_seq, const uint8_t **jpeg, uint32_t *len, uint32_t *seq);
void capture_release(void);

const capture_stats_t *capture_stats(void);

#endif /* CAPTURE_H */
