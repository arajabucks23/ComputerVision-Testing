#ifndef ETHERNETIF_H
#define ETHERNETIF_H

#include "lwip/err.h"
#include "lwip/netif.h"

/* Pass to netif_add(). Sets up the Ethernet MAC, DMA and the LAN8742 PHY. */
err_t ethernetif_init(struct netif *netif);

/* Call often from the main loop: hands received packets to lwIP, frees sent
 * packets and checks the cable link every 100 ms. */
void ethernetif_poll(struct netif *netif);

#endif /* ETHERNETIF_H */
