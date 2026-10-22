/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * robot_net.c
 * Network transport for the MQTT client.  See robot_net.h.
 */
#include <tk/tkernel.h>

#include "robot_net.h"

#if TM_WIFI_MQTT
#include "cyw43_utk.h"
#include "lwip_utk_link.h"

bool robot_net_link_up(void)
{
    T_CYW43_UTK_STATUS st;

    cyw43_utk_get_status(&st);
    return (st.link_up != 0U) && (st.dhcp_complete != 0U);
}

static int net_connect(void *ctx)
{
    (void)ctx;
    return lwip_utk_link_connect();
}

static int net_send(void *ctx, const uint8_t *data, uint16_t n)
{
    (void)ctx;
    return lwip_utk_link_send(data, n);
}

static int net_recv(void *ctx, uint8_t *buf, uint16_t cap)
{
    (void)ctx;
    return lwip_utk_link_recv(buf, cap);
}

static void net_close(void *ctx)
{
    (void)ctx;
    lwip_utk_link_close();
}

#else   /* no radio in this build */

bool robot_net_link_up(void)
{
    return false;
}

static int net_connect(void *ctx)
{
    (void)ctx;
    return -1;
}

static int net_send(void *ctx, const uint8_t *data, uint16_t n)
{
    (void)ctx;
    (void)data;
    (void)n;
    return -1;
}

static int net_recv(void *ctx, uint8_t *buf, uint16_t cap)
{
    (void)ctx;
    (void)buf;
    (void)cap;
    return -1;
}

static void net_close(void *ctx)
{
    (void)ctx;
}

#endif  /* TM_WIFI_MQTT */

mqtt_transport_t robot_net_transport(void)
{
    mqtt_transport_t t;

    t.tcp_connect = net_connect;
    t.tcp_send = net_send;
    t.tcp_recv = net_recv;
    t.tcp_close = net_close;
    t.ctx = NULL;
    return t;
}
