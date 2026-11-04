/*
 * Ethernet <-> lwIP glue for the NUCLEO-F746ZG (RMII, LAN8742 PHY), no RTOS.
 *
 * Adapted from ST's STM32CubeF7 examples:
 *   - STM32F769I_EVAL/LwIP_HTTP_Server_Raw          (no-RTOS structure)
 *   - STM32F746ZG-Nucleo/LwIP_HTTP_Server_Netconn_RTOS (RMII pins, LAN8742)
 * Copyright (c) 2016 STMicroelectronics, used under its license (see st_cubef7).
 *
 * Zero-copy in both directions:
 *   RX: the DMA writes straight into buffers from RX_POOL, handed to lwIP as pbufs.
 *   TX: the DMA reads straight from lwIP's pbufs; they are freed once sent.
 *
 * The data cache is off in this project, so no cache maintenance is needed.
 */
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "lan8742.h"

#define IFNAME0 's'
#define IFNAME1 't'

#define ETH_DMA_TRANSMIT_TIMEOUT    20U     /* ms */
#define ETH_RX_BUFFER_CNT           8U      /* must be > ETH_RX_DESC_CNT (4) */
#define LINK_CHECK_PERIOD_MS        100U

typedef struct
{
  struct pbuf_custom pbuf_custom;
  uint8_t buff[(ETH_RX_BUF_SIZE + 31) & ~31];
} RxBuff_t;

/* DMA descriptors and receive buffers live in SRAM2 (section .eth_dma, see the linker script). */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".eth_dma"), aligned(32)));
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".eth_dma"), aligned(32)));
extern u8_t memp_memory_RX_POOL_base[] __attribute__((section(".eth_dma")));
LWIP_MEMPOOL_DECLARE(RX_POOL, ETH_RX_BUFFER_CNT, sizeof(RxBuff_t), "Zero-copy RX PBUF pool");

ETH_HandleTypeDef heth;
static ETH_TxPacketConfig TxConfig;
static lan8742_Object_t LAN8742;
static uint8_t RxAllocFailed;
static uint8_t link_is_100m;        /* speed of the current link */
static uint32_t link_up_tick;       /* when the link last came up */
static uint8_t renego_tries;

#define RENEGOTIATE_AFTER_MS  3000U  /* a 10 Mbit/s link this long after coming up... */
#define RENEGOTIATE_MAX       5U     /* ...gets this many fresh negotiations */

static int32_t ETH_PHY_IO_Init(void);
static int32_t ETH_PHY_IO_DeInit(void);
static int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
static int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
static int32_t ETH_PHY_IO_GetTick(void);
static void pbuf_free_custom(struct pbuf *p);
static void ethernet_link_check_state(struct netif *netif);

static lan8742_IOCtx_t LAN8742_IOCtx = {ETH_PHY_IO_Init, ETH_PHY_IO_DeInit, ETH_PHY_IO_WriteReg,
                                        ETH_PHY_IO_ReadReg, ETH_PHY_IO_GetTick};

/* ---------------------------------------------------------------------------
 * lwIP -> hardware
 * ------------------------------------------------------------------------- */
static void low_level_init(struct netif *netif)
{
  static uint8_t macaddress[6] = {ETH_MAC_ADDR0, ETH_MAC_ADDR1, ETH_MAC_ADDR2,
                                  ETH_MAC_ADDR3, ETH_MAC_ADDR4, ETH_MAC_ADDR5};

  heth.Instance = ETH;
  heth.Init.MACAddr = macaddress;
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxBuffLen = ETH_RX_BUF_SIZE;
  HAL_ETH_Init(&heth);

  netif->hwaddr_len = ETH_HWADDR_LEN;
  memcpy(netif->hwaddr, macaddress, 6);
  netif->mtu = ETH_MAX_PAYLOAD;
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

  LWIP_MEMPOOL_INIT(RX_POOL);

  memset(&TxConfig, 0, sizeof(TxConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

  LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);
  if (LAN8742_Init(&LAN8742) != LAN8742_STATUS_OK)
  {
    printf("[eth] PHY (LAN8742) not found\r\n");
    netif_set_link_down(netif);
    netif_set_down(netif);
    return;
  }
  ethernet_link_check_state(netif);
}

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];
  uint32_t i = 0U;
  (void)netif;

  memset(Txbuffer, 0, sizeof(Txbuffer));
  for (struct pbuf *q = p; q != NULL; q = q->next)
  {
    if (i >= ETH_TX_DESC_CNT)
      return ERR_IF;
    Txbuffer[i].buffer = q->payload;
    Txbuffer[i].len = q->len;
    if (i > 0)
      Txbuffer[i - 1].next = &Txbuffer[i];
    i++;
  }

  TxConfig.Length = p->tot_len;
  TxConfig.TxBuffer = Txbuffer;
  TxConfig.pData = p;

  /* Keep the pbuf alive until the DMA has sent it: HAL_ETH_ReleaseTxPacket() later
   * calls HAL_ETH_TxFreeCallback(), which drops this reference.
   * Note: the HAL's blocking HAL_ETH_Transmit() does not record the packet, so it
   * would never be released (each reply leaked a buffer). Use the _IT variant;
   * it needs no interrupt, completion is picked up by HAL_ETH_ReleaseTxPacket(). */
  pbuf_ref(p);
  uint32_t start = HAL_GetTick();
  while (HAL_ETH_Transmit_IT(&heth, &TxConfig) != HAL_OK)
  {
    if (!(HAL_ETH_GetError(&heth) & HAL_ETH_ERROR_BUSY) ||
        HAL_GetTick() - start > ETH_DMA_TRANSMIT_TIMEOUT)
    {
      pbuf_free(p);   /* not queued: drop our reference */
      return ERR_IF;
    }
    /* All TX descriptors in use: free the ones the DMA has finished, then retry. */
    HAL_ETH_ReleaseTxPacket(&heth);
  }
  return ERR_OK;
}

static struct pbuf *low_level_input(struct netif *netif)
{
  struct pbuf *p = NULL;
  (void)netif;

  if (!RxAllocFailed)
    HAL_ETH_ReadData(&heth, (void **)&p);
  return p;
}

static void ethernetif_input(struct netif *netif)
{
  struct pbuf *p;

  while ((p = low_level_input(netif)) != NULL)
  {
    if (netif->input(p, netif) != ERR_OK)
      pbuf_free(p);
  }
}

err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;
  low_level_init(netif);
  return ERR_OK;
}

void ethernetif_poll(struct netif *netif)
{
  static uint32_t last_link_check = 0;

  ethernetif_input(netif);
  HAL_ETH_ReleaseTxPacket(&heth);

  if (HAL_GetTick() - last_link_check >= LINK_CHECK_PERIOD_MS)
  {
    last_link_check = HAL_GetTick();
    ethernet_link_check_state(netif);

    /* With the board's supply sagging, the PHY sometimes settles on 10 Mbit/s
     * at power-up (too slow for the video). Ask it to negotiate again once the
     * supply has settled -- the same thing that fixed it on the boots where it
     * recovered by itself. ST's LAN8742_StartAutoNego() only sets the enable
     * bit, so set "restart auto-negotiation" (BCR bit 9) directly. */
    if (netif_is_link_up(netif) && !link_is_100m && renego_tries < RENEGOTIATE_MAX &&
        HAL_GetTick() - link_up_tick >= RENEGOTIATE_AFTER_MS)
    {
      uint32_t bcr;
      if (ETH_PHY_IO_ReadReg(LAN8742.DevAddr, LAN8742_BCR, &bcr) == 0)
      {
        ETH_PHY_IO_WriteReg(LAN8742.DevAddr, LAN8742_BCR,
                            bcr | LAN8742_BCR_AUTONEGO_EN | LAN8742_BCR_RESTART_AUTONEGO);
        renego_tries++;
        link_up_tick = HAL_GetTick();
        printf("[eth] link is 10 Mbit/s: restarting negotiation (try %u of %u)\r\n",
               renego_tries, RENEGOTIATE_MAX);
      }
    }
  }
}

/* lwIP's clock (NO_SYS mode). */
u32_t sys_now(void)
{
  return HAL_GetTick();
}

/* ---------------------------------------------------------------------------
 * Cable link up/down: follows the PHY and restarts the MAC at the new speed.
 * ------------------------------------------------------------------------- */
static void ethernet_link_check_state(struct netif *netif)
{
  ETH_MACConfigTypeDef MACConf = {0};
  int32_t state = LAN8742_GetLinkState(&LAN8742);
  uint32_t speed = 0U, duplex = 0U;

  if (netif_is_link_up(netif) && state <= LAN8742_STATUS_LINK_DOWN)
  {
    HAL_ETH_Stop(&heth);
    netif_set_down(netif);
    netif_set_link_down(netif);
    printf("[eth] link down\r\n");
    return;
  }
  if (netif_is_link_up(netif) || state <= LAN8742_STATUS_LINK_DOWN)
    return;

  switch (state)
  {
    case LAN8742_STATUS_100MBITS_FULLDUPLEX: speed = ETH_SPEED_100M; duplex = ETH_FULLDUPLEX_MODE; break;
    case LAN8742_STATUS_100MBITS_HALFDUPLEX: speed = ETH_SPEED_100M; duplex = ETH_HALFDUPLEX_MODE; break;
    case LAN8742_STATUS_10MBITS_FULLDUPLEX:  speed = ETH_SPEED_10M;  duplex = ETH_FULLDUPLEX_MODE; break;
    case LAN8742_STATUS_10MBITS_HALFDUPLEX:  speed = ETH_SPEED_10M;  duplex = ETH_HALFDUPLEX_MODE; break;
    default: return;  /* auto-negotiation not finished yet */
  }

  HAL_ETH_GetMACConfig(&heth, &MACConf);
  MACConf.DuplexMode = duplex;
  MACConf.Speed = speed;
  HAL_ETH_SetMACConfig(&heth, &MACConf);
  HAL_ETH_Start(&heth);
  netif_set_up(netif);
  netif_set_link_up(netif);
  link_is_100m = (speed == ETH_SPEED_100M);
  link_up_tick = HAL_GetTick();
  printf("[eth] link up, %s Mbit/s %s duplex\r\n",
         speed == ETH_SPEED_100M ? "100" : "10",
         duplex == ETH_FULLDUPLEX_MODE ? "full" : "half");
}

/* ---------------------------------------------------------------------------
 * HAL callbacks: RX buffer allocation and TX completion
 * ------------------------------------------------------------------------- */
void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
  struct pbuf_custom *p = LWIP_MEMPOOL_ALLOC(RX_POOL);

  if (p)
  {
    *buff = (uint8_t *)p + offsetof(RxBuff_t, buff);
    p->custom_free_function = pbuf_free_custom;
    pbuf_alloced_custom(PBUF_RAW, 0, PBUF_REF, p, *buff, ETH_RX_BUF_SIZE);
  }
  else
  {
    RxAllocFailed = 1;
    *buff = NULL;
  }
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
  struct pbuf **ppStart = (struct pbuf **)pStart;
  struct pbuf **ppEnd = (struct pbuf **)pEnd;
  struct pbuf *p = (struct pbuf *)(buff - offsetof(RxBuff_t, buff));

  p->next = NULL;
  p->tot_len = 0;
  p->len = Length;

  if (!*ppStart)
    *ppStart = p;
  else
    (*ppEnd)->next = p;
  *ppEnd = p;

  for (p = *ppStart; p != NULL; p = p->next)
    p->tot_len += Length;
}

void HAL_ETH_TxFreeCallback(uint32_t *buff)
{
  pbuf_free((struct pbuf *)buff);
}

static void pbuf_free_custom(struct pbuf *p)
{
  LWIP_MEMPOOL_FREE(RX_POOL, (struct pbuf_custom *)p);
  RxAllocFailed = 0;  /* a buffer is free again: let reception continue */
}

/* ---------------------------------------------------------------------------
 * Ethernet pins (RMII on the Nucleo, all AF11) and clock
 * ------------------------------------------------------------------------- */
void HAL_ETH_MspInit(ETH_HandleTypeDef *h)
{
  GPIO_InitTypeDef g = {0};
  (void)h;

  /*  RMII_REF_CLK PA1   RMII_MDIO PA2   RMII_CRS_DV PA7
      RMII_MDC     PC1   RMII_RXD0 PC4   RMII_RXD1   PC5
      RMII_TX_EN   PG11  RMII_TXD0 PG13  RMII_TXD1   PB13 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF11_ETH;

  g.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &g);
  g.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &g);
  g.Pin = GPIO_PIN_11 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOG, &g);

  __HAL_RCC_ETH_CLK_ENABLE();
}

/* ---------------------------------------------------------------------------
 * PHY register access over MDIO (used by the LAN8742 driver)
 * ------------------------------------------------------------------------- */
static int32_t ETH_PHY_IO_Init(void)
{
  HAL_ETH_SetMDIOClockRange(&heth);
  return 0;
}

static int32_t ETH_PHY_IO_DeInit(void)
{
  return 0;
}

static int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
  return HAL_ETH_ReadPHYRegister(&heth, DevAddr, RegAddr, pRegVal) == HAL_OK ? 0 : -1;
}

static int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
  return HAL_ETH_WritePHYRegister(&heth, DevAddr, RegAddr, RegVal) == HAL_OK ? 0 : -1;
}

static int32_t ETH_PHY_IO_GetTick(void)
{
  return (int32_t)HAL_GetTick();
}
