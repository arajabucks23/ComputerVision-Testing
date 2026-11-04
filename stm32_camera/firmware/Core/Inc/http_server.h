/* HTTP server on port 80 (lwIP raw API): /stream (MJPEG) and /log. */
#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include <stdint.h>

void http_server_init(void);
void http_server_poll(void);        /* call from the main loop */
uint32_t http_frames_sent(void);    /* frames fully handed to the network */

#endif /* HTTP_SERVER_H */
