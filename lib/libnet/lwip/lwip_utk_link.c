/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * lwip_utk_link.c
 * Raw-TCP client link between the MQTT client (application tasks) and lwIP
 * (radio owner task).  See lwip_utk_link.h.
 *
 * Threading: the application side and the service side exchange data through
 * single-producer/single-consumer rings and a few word-sized request flags.
 * Both sides run on the same RP2040 core (the robot tasks are pinned to
 * processor 1 in the SMP profile), so compiler barriers are enough; a
 * different core assignment would need hardware barriers here.
 *
 * NOT YET RUN ON THE CAR: it compiles against the Pico SDK 2.2.0 lwIP, but
 * connect/reconnect behaviour has not been exercised against a real broker.
 */
#include <stddef.h>
#include <stdint.h>

#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"

#include "lwip_utk_link.h"
#include "mqtt_config.h"

#define LINK_IDLE           (0U)
#define LINK_CONNECTING     (1U)
#define LINK_CONNECTED      (2U)
#define LINK_FAILED         (3U)

#define LINK_CONNECT_TIMEOUT_MS (8000U)

#define BARRIER()           __asm__ volatile("" ::: "memory")

_Static_assert((LWIP_UTK_LINK_TX_RING & (LWIP_UTK_LINK_TX_RING - 1U)) == 0U,
               "the TX ring size must be a power of two");
_Static_assert((LWIP_UTK_LINK_RX_RING & (LWIP_UTK_LINK_RX_RING - 1U)) == 0U,
               "the RX ring size must be a power of two");

static volatile uint32_t link_state = LINK_IDLE;
static volatile uint32_t want_open;
static volatile uint32_t want_close;

static struct tcp_pcb *pcb;
static ip4_addr_t broker;
static uint32_t connect_started_ms;

static uint8_t tx_ring[LWIP_UTK_LINK_TX_RING];
static volatile uint32_t tx_head;               /* written by the application */
static volatile uint32_t tx_tail;               /* written by the service */
static uint8_t rx_ring[LWIP_UTK_LINK_RX_RING];
static volatile uint32_t rx_head;               /* written by the service */
static volatile uint32_t rx_tail;               /* written by the application */

static T_LWIP_UTK_LINK_STATUS status;

/* ---- application side ---------------------------------------------------- */

int lwip_utk_link_connect(void)
{
    if (link_state == LINK_CONNECTED) {
        return 1;
    }
    if (link_state == LINK_FAILED) {
        return -1;
    }
    want_open = 1U;                             /* idle or still connecting */
    BARRIER();
    return 0;
}

int lwip_utk_link_send(const uint8_t *data, uint16_t len)
{
    uint32_t head = tx_head;
    uint32_t free_bytes;
    uint16_t i;

    if (link_state != LINK_CONNECTED) {
        return -1;
    }
    free_bytes = LWIP_UTK_LINK_TX_RING - (head - tx_tail);
    if (len > free_bytes) {
        status.tx_dropped++;
        return 0;                               /* all or nothing: never a partial packet */
    }
    for (i = 0U; i < len; i++) {
        tx_ring[(head + i) & (LWIP_UTK_LINK_TX_RING - 1U)] = data[i];
    }
    BARRIER();
    tx_head = head + len;
    return (int)len;
}

int lwip_utk_link_recv(uint8_t *buf, uint16_t cap)
{
    uint32_t tail = rx_tail;
    uint32_t avail = rx_head - tail;
    uint16_t n = (avail > cap) ? cap : (uint16_t)avail;
    uint16_t i;

    if (n == 0U) {
        return (link_state == LINK_CONNECTED) ? 0 : -1;
    }
    for (i = 0U; i < n; i++) {
        buf[i] = rx_ring[(tail + i) & (LWIP_UTK_LINK_RX_RING - 1U)];
    }
    BARRIER();
    rx_tail = tail + n;
    return (int)n;
}

void lwip_utk_link_close(void)
{
    want_open = 0U;
    want_close = 1U;
    BARRIER();
}

void lwip_utk_link_get_status(T_LWIP_UTK_LINK_STATUS *out)
{
    if (out != NULL) {
        *out = status;
        out->state = link_state;
    }
}

/* ---- service side (radio owner task) --------------------------------------- */

static void detach(struct tcp_pcb *p)
{
    tcp_arg(p, NULL);
    tcp_recv(p, NULL);
    tcp_sent(p, NULL);
    tcp_err(p, NULL);
    tcp_poll(p, NULL, 0);
}

static void reset_rings(void)
{
    tx_head = 0U;
    tx_tail = 0U;
    rx_head = 0U;
    rx_tail = 0U;
}

/* lwIP has already freed the pcb when this runs. */
static void link_err(void *arg, err_t err)
{
    (void)arg;
    pcb = NULL;
    status.last_error = err;
    status.failures++;
    link_state = LINK_FAILED;
}

static err_t link_recv(void *arg, struct tcp_pcb *p, struct pbuf *buf, err_t err)
{
    uint16_t offset = 0U;

    (void)arg;
    if ((err != ERR_OK) || (buf == NULL)) {     /* error, or an orderly close from the peer */
        if (buf != NULL) {
            pbuf_free(buf);
        }
        status.last_error = err;
        status.failures++;
        link_state = LINK_FAILED;
        return ERR_OK;
    }
    tcp_recved(p, buf->tot_len);
    while (offset < buf->tot_len) {
        uint32_t head = rx_head;
        uint32_t free_bytes = LWIP_UTK_LINK_RX_RING - (head - rx_tail);
        uint16_t chunk = (uint16_t)(buf->tot_len - offset);

        if (free_bytes == 0U) {
            status.rx_overflow += (uint32_t)(buf->tot_len - offset);
            break;                              /* the application is not keeping up */
        }
        if (chunk > free_bytes) {
            chunk = (uint16_t)free_bytes;
        }
        {
            uint8_t tmp[64];
            uint16_t done = 0U;

            while (done < chunk) {
                uint16_t part = (uint16_t)(chunk - done);
                uint16_t k;

                if (part > sizeof(tmp)) {
                    part = (uint16_t)sizeof(tmp);
                }
                if (pbuf_copy_partial(buf, tmp, part, (u16_t)(offset + done)) != part) {
                    break;
                }
                for (k = 0U; k < part; k++) {
                    rx_ring[(head + done + k) & (LWIP_UTK_LINK_RX_RING - 1U)] = tmp[k];
                }
                done = (uint16_t)(done + part);
            }
            BARRIER();
            rx_head = head + done;
            status.bytes_received += done;
            offset = (uint16_t)(offset + chunk);
        }
    }
    pbuf_free(buf);
    return ERR_OK;
}

static err_t link_connected(void *arg, struct tcp_pcb *p, err_t err)
{
    (void)arg;
    (void)p;
    if (err != ERR_OK) {
        status.last_error = err;
        status.failures++;
        link_state = LINK_FAILED;
        return ERR_OK;
    }
    status.connects++;
    link_state = LINK_CONNECTED;
    return ERR_OK;
}

static void release(void)
{
    if (pcb != NULL) {
        detach(pcb);
        if (tcp_close(pcb) != ERR_OK) {
            tcp_abort(pcb);
        }
        pcb = NULL;
    }
    reset_rings();
    link_state = LINK_IDLE;
}

static void begin_connect(struct netif *netif)
{
    err_t result;

    if (!ip4addr_aton(UTK_MQTT_BROKER_ADDRESS, &broker) || ip4_addr_isany_val(broker)) {
        status.last_error = ERR_ARG;
        status.failures++;
        link_state = LINK_FAILED;
        return;
    }
    pcb = tcp_new();
    if (pcb == NULL) {
        status.last_error = ERR_MEM;
        status.failures++;
        link_state = LINK_FAILED;
        return;
    }
    reset_rings();
    tcp_bind_netif(pcb, netif);
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, link_recv);
    tcp_err(pcb, link_err);
    connect_started_ms = sys_now();
    link_state = LINK_CONNECTING;
    result = tcp_connect(pcb, (const ip_addr_t *)&broker, (uint16_t)UTK_MQTT_BROKER_PORT,
                         link_connected);
    if (result != ERR_OK) {
        detach(pcb);
        tcp_abort(pcb);
        pcb = NULL;
        status.last_error = result;
        status.failures++;
        link_state = LINK_FAILED;
    }
}

/* Drain the TX ring into lwIP. */
static void pump_tx(void)
{
    uint32_t tail = tx_tail;
    uint32_t pending = tx_head - tail;

    while ((pending > 0U) && (pcb != NULL) && (link_state == LINK_CONNECTED)) {
        uint8_t tmp[64];
        uint16_t room = tcp_sndbuf(pcb);
        uint16_t n = (pending > sizeof(tmp)) ? (uint16_t)sizeof(tmp) : (uint16_t)pending;
        uint16_t i;

        if (room == 0U) {
            break;                              /* lwIP's send buffer is full: try next poll */
        }
        if (n > room) {
            n = room;
        }
        for (i = 0U; i < n; i++) {
            tmp[i] = tx_ring[(tail + i) & (LWIP_UTK_LINK_TX_RING - 1U)];
        }
        if (tcp_write(pcb, tmp, n, TCP_WRITE_FLAG_COPY) != ERR_OK) {
            break;
        }
        tail += n;
        pending -= n;
        status.bytes_sent += n;
        BARRIER();
        tx_tail = tail;
    }
    if (pcb != NULL) {
        (void)tcp_output(pcb);
    }
}

void lwip_utk_link_poll(struct netif *netif)
{
    const uint32_t now = sys_now();

    if (want_close != 0U) {
        want_close = 0U;
        release();
        return;
    }
    if ((link_state == LINK_FAILED) && (pcb != NULL)) {
        release();
        link_state = LINK_FAILED;               /* stay failed until the application closes */
        return;
    }
    if ((netif == NULL) || !netif_is_up(netif) || !netif_is_link_up(netif) ||
        ip4_addr_isany_val(*netif_ip4_addr(netif))) {
        if (link_state != LINK_IDLE) {          /* the network went away under a live link */
            release();
            link_state = LINK_FAILED;
            status.failures++;
        }
        return;
    }

    switch (link_state) {
    case LINK_IDLE:
        if (want_open != 0U) {
            begin_connect(netif);
        }
        break;

    case LINK_CONNECTING:
        if ((now - connect_started_ms) >= LINK_CONNECT_TIMEOUT_MS) {
            status.last_error = ERR_TIMEOUT;
            status.failures++;
            release();
            link_state = LINK_FAILED;
        }
        break;

    case LINK_CONNECTED:
        pump_tx();
        break;

    default:
        break;                                  /* FAILED: waiting for lwip_utk_link_close() */
    }
}
