/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * lwip_utk_link.h
 * A general-purpose raw-TCP client link for the robot's MQTT client.
 *
 * lwIP runs in NO_SYS mode inside the radio owner task, so application tasks
 * must not call it.  This link is the bridge: the application side (any task)
 * asks for a connection and moves bytes through two small rings; the service
 * side (lwip_utk_link_poll, called from the radio loop) owns the pcb and does
 * the lwIP work.  Nothing on the application side ever blocks.
 *
 * Application side, for the MQTT client's transport interface:
 *   lwip_utk_link_connect()  1 connected, 0 in progress, <0 failed
 *   lwip_utk_link_send()     bytes accepted, 0 if the ring is full, <0 if down
 *   lwip_utk_link_recv()     bytes read, 0 if none, <0 if the link closed
 *   lwip_utk_link_close()
 */
#ifndef LWIP_UTK_LINK_H
#define LWIP_UTK_LINK_H

#include <stdint.h>

struct netif;

#define LWIP_UTK_LINK_TX_RING   (512U)
#define LWIP_UTK_LINK_RX_RING   (512U)

/* ---- application side ---- */
int lwip_utk_link_connect(void);
int lwip_utk_link_send(const uint8_t *data, uint16_t len);
int lwip_utk_link_recv(uint8_t *buf, uint16_t cap);
void lwip_utk_link_close(void);

/* ---- service side: call from the radio owner task, once per poll ---- */
void lwip_utk_link_poll(struct netif *netif);

typedef struct {
    uint32_t state;             /* 0 idle, 1 connecting, 2 connected, 3 failed */
    uint32_t connects;
    uint32_t failures;
    uint32_t bytes_sent;
    uint32_t bytes_received;
    uint32_t tx_dropped;        /* send attempts refused because the ring was full */
    uint32_t rx_overflow;       /* received bytes lost because the app was slow */
    int32_t  last_error;
} T_LWIP_UTK_LINK_STATUS;

void lwip_utk_link_get_status(T_LWIP_UTK_LINK_STATUS *status);

#endif /* LWIP_UTK_LINK_H */
