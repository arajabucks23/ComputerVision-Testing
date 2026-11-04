/*
 * HTTP server on port 80, lwIP raw API (everything runs from the main loop).
 *
 *   GET / or /stream -> MJPEG: multipart/x-mixed-replace, one JPEG per part.
 *                       Opens in a browser, VLC and cv2.VideoCapture(url).
 *   GET /status      -> stream/capture counters as text
 *   GET /log         -> the board's log as text/plain
 *
 * One stream client at a time: a new viewer replaces the old one, and a client that
 * takes no data for 5 s is dropped (so a dead connection never blocks the stream).
 * Each frame is copied into
 * lwIP's send buffer as space frees up (tcp_sent callback / main loop), so the
 * frame buffer is only held while it is being queued, not until the PC acks it.
 */
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "lwip/tcp.h"
#include "http_server.h"
#include "capture.h"
#include "dht11.h"
#include "adc.h"
#include "log.h"

typedef struct
{
  struct tcp_pcb *pcb;
  const uint8_t *jpeg;        /* frame being sent (locked in capture.c), NULL = none */
  uint32_t len, off;
  uint32_t seq;               /* last frame sequence number sent */
  char part_hdr[256];
  uint32_t hdr_len, hdr_off;
  uint8_t trailer_sent;
} stream_t;

static stream_t stream;       /* the single stream client */
static uint32_t frames_sent;
static uint32_t last_progress;   /* HAL tick of the last data accepted/acked on the stream */
static uint32_t stream_kicked, stream_stalled;

#define STALL_TIMEOUT_MS 5000U   /* stream client that takes no data for this long is dropped */

static const char STREAM_HEADER[] =
  "HTTP/1.0 200 OK\r\n"
  "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
  "Cache-Control: no-cache\r\n"
  "Connection: close\r\n\r\n";

/* ---------------------------------------------------------------------------
 * Streaming
 * ------------------------------------------------------------------------- */
static void stream_end(void)
{
  if (stream.jpeg)
    capture_release();
  memset(&stream, 0, sizeof(stream));
}

/* Drop the current stream client right away (RST), e.g. dead or replaced by a new viewer. */
static void stream_abort(void)
{
  struct tcp_pcb *pcb = stream.pcb;

  if (pcb == NULL)
    return;
  tcp_arg(pcb, NULL);
  tcp_recv(pcb, NULL);
  tcp_sent(pcb, NULL);
  tcp_err(pcb, NULL);
  stream_end();
  tcp_abort(pcb);
}

/* Queue as much as lwIP will take: part header, JPEG bytes, "\r\n"; then the next frame. */
static void stream_pump(void)
{
  struct tcp_pcb *pcb = stream.pcb;

  if (pcb == NULL)
    return;
  for (;;)
  {
    if (stream.jpeg == NULL)
    {
      uint32_t seq;
      if (!capture_take(stream.seq, &stream.jpeg, &stream.len, &seq))
        break;                                   /* no newer frame yet */
      stream.seq = seq;
      stream.off = 0;
      stream.hdr_off = 0;
      stream.trailer_sent = 0;
      /* The latest DHT11 reading rides along as extra part headers (players ignore them). */
      char sensor[160] = "";
      dht11_reading_t r;
      uint32_t age;
      int n = 0;
      co_reading_t co;
      if (dht11_last(&r, &age))
        n = snprintf(sensor, sizeof(sensor), "X-Temperature-C: %u.%u\r\nX-Humidity-Pct: %u\r\nX-Sensor-Age-Ms: %lu\r\n",
                     r.temperature, r.temperature_dec, r.humidity, (unsigned long)age);
      if (co_sensor_last(&co, &age) && age < 5000 && n >= 0 && n < (int)sizeof(sensor))
      {
        if (co.warm)
          snprintf(sensor + n, sizeof(sensor) - n, "X-CO-mV: %lu\r\nX-CO-ppm: %lu.%lu\r\n", (unsigned long)co.mv,
                   (unsigned long)co.ppm, (unsigned long)((uint32_t)(co.ppm * 10) % 10));
        else
          snprintf(sensor + n, sizeof(sensor) - n, "X-CO-mV: %lu\r\nX-CO-ppm: warming\r\n", (unsigned long)co.mv);
      }
      stream.hdr_len = (uint32_t)snprintf(stream.part_hdr, sizeof(stream.part_hdr),
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %lu\r\n%s\r\n", (unsigned long)stream.len, sensor);
    }

    uint32_t room = tcp_sndbuf(pcb);
    if (room == 0 || tcp_sndqueuelen(pcb) >= TCP_SND_QUEUELEN - 4)
      break;

    if (stream.hdr_off < stream.hdr_len)
    {
      uint32_t n = stream.hdr_len - stream.hdr_off;
      if (n > room) n = room;
      if (tcp_write(pcb, stream.part_hdr + stream.hdr_off, (u16_t)n, TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE) != ERR_OK)
        break;
      stream.hdr_off += n;
      last_progress = HAL_GetTick();
    }
    else if (stream.off < stream.len)
    {
      uint32_t n = stream.len - stream.off;
      if (n > room) n = room;
      if (tcp_write(pcb, stream.jpeg + stream.off, (u16_t)n, TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE) != ERR_OK)
        break;
      stream.off += n;
      last_progress = HAL_GetTick();
    }
    else if (!stream.trailer_sent)
    {
      if (room < 2 || tcp_write(pcb, "\r\n", 2, 0) != ERR_OK)
        break;
      stream.trailer_sent = 1;
    }
    else
    {
      capture_release();       /* whole frame copied into lwIP: buffer free for capture */
      stream.jpeg = NULL;
      frames_sent++;
    }
  }
  tcp_output(pcb);
}

static err_t stream_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
  (void)arg; (void)pcb; (void)len;
  last_progress = HAL_GetTick();
  stream_pump();
  return ERR_OK;
}

static void stream_err(void *arg, err_t err)
{
  (void)arg; (void)err;
  stream.pcb = NULL;           /* lwIP already freed the pcb */
  stream_end();
}

/* ---------------------------------------------------------------------------
 * Requests
 * ------------------------------------------------------------------------- */
static void write_piece(const char *data, uint32_t len, void *ctx)
{
  if (len > 0)
    tcp_write((struct tcp_pcb *)ctx, data, (u16_t)len, TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE);
}

static void answer_and_close(struct tcp_pcb *pcb, const char *status_and_headers, int with_log)
{
  tcp_write(pcb, status_and_headers, (u16_t)strlen(status_and_headers), TCP_WRITE_FLAG_MORE);
  if (with_log)
    log_for_each(write_piece, pcb);
  tcp_output(pcb);
  tcp_close(pcb);
}

static err_t on_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
  char req[32] = {0};
  (void)err;

  if (p == NULL)                 /* client closed */
  {
    if (pcb == stream.pcb)
    {
      tcp_err(pcb, NULL);
      tcp_sent(pcb, NULL);
      stream_end();
    }
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_close(pcb);
    return ERR_OK;
  }
  pbuf_copy_partial(p, req, sizeof(req) - 1, 0);
  tcp_recved(pcb, p->tot_len);
  pbuf_free(p);
  if (arg != NULL)               /* request already handled; ignore further data */
    return ERR_OK;
  tcp_arg(pcb, pcb);

  if (strncmp(req, "GET /log", 8) == 0)
  {
    answer_and_close(pcb, "HTTP/1.0 200 OK\r\nContent-Type: text/plain; charset=utf-8\r\n"
                          "Connection: close\r\n\r\n", 1);
  }
  else if (strncmp(req, "GET /status", 11) == 0)
  {
    static char text[640];
    const capture_stats_t *s = capture_stats();
    struct tcp_pcb *sp = stream.pcb;
    int n = snprintf(text, sizeof(text),
      "HTTP/1.0 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n"
      "uptime_s %lu\r\n"
      "stream_client %s\r\n"
      "stream_tcp_state %d  sndbuf %u  queuelen %u  unacked %s\r\n"
      "stream_frame %lu  offset %lu of %lu  ms_since_progress %lu\r\n"
      "frames_sent %lu  clients_replaced %lu  clients_stalled %lu\r\n"
      "captured_ok %lu  bad %lu  too_big %lu  skipped %lu  dcmi_errors %lu  timeouts %lu\r\n",
      (unsigned long)(HAL_GetTick() / 1000),
      sp ? ipaddr_ntoa(&sp->remote_ip) : "none",
      sp ? (int)sp->state : -1, sp ? (unsigned)tcp_sndbuf(sp) : 0u,
      sp ? (unsigned)tcp_sndqueuelen(sp) : 0u, (sp && sp->unacked) ? "yes" : "no",
      (unsigned long)stream.seq, (unsigned long)stream.off, (unsigned long)stream.len,
      (unsigned long)(sp ? HAL_GetTick() - last_progress : 0),
      (unsigned long)frames_sent, (unsigned long)stream_kicked, (unsigned long)stream_stalled,
      (unsigned long)s->frames_ok, (unsigned long)s->frames_bad, (unsigned long)s->frames_too_big,
      (unsigned long)s->frames_skipped, (unsigned long)s->dcmi_errors, (unsigned long)s->timeouts);
    tcp_write(pcb, text, (u16_t)n, TCP_WRITE_FLAG_COPY);
    tcp_output(pcb);
    tcp_close(pcb);
  }
  else if (strncmp(req, "GET / ", 6) == 0 || strncmp(req, "GET /stream", 11) == 0)
  {
    /* Newest viewer wins: a dead or forgotten connection can never lock the stream. */
    if (stream.pcb != NULL)
    {
      stream_abort();
      stream_kicked++;
    }
    stream.pcb = pcb;
    last_progress = HAL_GetTick();
    tcp_err(pcb, stream_err);
    tcp_sent(pcb, stream_sent);
    tcp_nagle_disable(pcb);
    tcp_write(pcb, STREAM_HEADER, sizeof(STREAM_HEADER) - 1, 0);
    stream_pump();
  }
  else
  {
    answer_and_close(pcb, "HTTP/1.0 404 Not Found\r\nContent-Type: text/plain\r\n\r\n"
                          "Try /stream, /status or /log\r\n", 0);
  }
  return ERR_OK;
}

static err_t on_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  (void)arg;
  if (err != ERR_OK || pcb == NULL)
    return ERR_VAL;
  tcp_arg(pcb, NULL);
  tcp_recv(pcb, on_recv);
  return ERR_OK;
}

void http_server_init(void)
{
  struct tcp_pcb *pcb = tcp_new();

  tcp_bind(pcb, IP_ADDR_ANY, 80);
  pcb = tcp_listen(pcb);
  tcp_accept(pcb, on_accept);
}

void http_server_poll(void)
{
  stream_pump();               /* pick up a new frame as soon as one is ready */

  /* Watchdog: a client that has taken no data for 5 s is dead or stuck; drop it. */
  if (stream.pcb != NULL && HAL_GetTick() - last_progress > STALL_TIMEOUT_MS)
  {
    printf("[http] stream client stalled (state %d, sndbuf %u, queuelen %u, frame offset %lu/%lu): dropped\r\n",
           (int)stream.pcb->state, (unsigned)tcp_sndbuf(stream.pcb), (unsigned)tcp_sndqueuelen(stream.pcb),
           (unsigned long)stream.off, (unsigned long)stream.len);
    stream_stalled++;
    stream_abort();
  }
}

uint32_t http_frames_sent(void)
{
  return frames_sent;
}
