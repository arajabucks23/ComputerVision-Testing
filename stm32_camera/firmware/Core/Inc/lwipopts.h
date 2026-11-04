/*
 * lwIP settings for the camera node.
 *
 * No RTOS: lwIP runs from the main loop (NO_SYS = 1), using the "raw" API.
 * Based on ST's LwIP_HTTP_Server_Raw example, with TCP tuned for streaming
 * large JPEG frames (full 1460-byte segments, 8 segments in flight).
 */
#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS                      1
#define SYS_LIGHTWEIGHT_PROT        0   /* lwIP is only ever called from the main loop */
#define LWIP_NETCONN                0
#define LWIP_SOCKET                 0

/* ---------- Memory ---------- */
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    (16 * 1024) /* lwIP heap: packet headers, ARP/ICMP replies */
#define MEMP_NUM_PBUF               32          /* pbufs that point at frame data (no copy) */
#define MEMP_NUM_TCP_PCB            4
#define MEMP_NUM_TCP_PCB_LISTEN     2
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_SYS_TIMEOUT        10
#define PBUF_POOL_SIZE              4           /* RX uses the driver's own zero-copy pool */
#define PBUF_POOL_BUFSIZE           1524

/* ---------- IP / ICMP / UDP ---------- */
#define LWIP_IPV4                   1
#define LWIP_ICMP                   1           /* answers ping */
#define LWIP_DHCP                   0           /* static IP 192.168.10.2 */
#define LWIP_UDP                    1
#define UDP_TTL                     255

/* ---------- TCP ---------- */
#define LWIP_TCP                    1
#define TCP_TTL                     255
#define TCP_QUEUE_OOSEQ             0
#define TCP_MSS                     1460                        /* full Ethernet frames */
#define TCP_SND_BUF                 (8 * TCP_MSS)               /* ~11.7 KB in flight */
#define TCP_SND_QUEUELEN            (4 * TCP_SND_BUF / TCP_MSS) /* header + data pbuf per segment */
#define TCP_WND                     (4 * TCP_MSS)

/* ---------- Misc ---------- */
#define LWIP_STATS                  0
#define LWIP_NETIF_LINK_CALLBACK    0

/* ---------- Checksums: the F7 Ethernet MAC computes them in hardware ---------- */
#define CHECKSUM_GEN_IP             0
#define CHECKSUM_GEN_UDP            0
#define CHECKSUM_GEN_TCP            0
#define CHECKSUM_GEN_ICMP           0
#define CHECKSUM_CHECK_IP           0
#define CHECKSUM_CHECK_UDP          0
#define CHECKSUM_CHECK_TCP          0
#define CHECKSUM_CHECK_ICMP         0

#endif /* LWIPOPTS_H */
