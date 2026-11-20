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

/*
 * With WIFI_MQTT=1 the MQTT session lives inside the radio owner task (lib/libnet/lwip/mqtt_lwip.c, with the
 * WiFi/DHCP recovery and diagnostics).  The robot does not run its own client: net_task in robot_tasks.c hands
 * its telemetry records to that session with telemetry_robot_submit().  The application-level MQTT client
 * (mqtt_client.c) and its TCP link (lwip_utk_link.c) are therefore not used in this build, and this transport
 * only reports failure.
 */
static robot_net_info_t g_info;                 /* written by robot_net_refresh() only */
static T_CYW43_UTK_STATUS g_status;             /* static: 2576 bytes, and only robot_net_refresh() uses it */

static int16_t sat16(int32_t v)
{
    return (v > 32767) ? (int16_t)32767 : ((v < -32768) ? (int16_t)-32768 : (int16_t)v);
}

void robot_net_refresh(void)
{
    robot_net_info_t n;
    uint32_t i;

    cyw43_utk_get_status(&g_status);
    n.ready = (g_status.ready != 0U) ? 1U : 0U;
    n.link_up = (g_status.link_up != 0U) ? 1U : 0U;
    n.dhcp = (g_status.dhcp_complete != 0U) ? 1U : 0U;
    n.mqtt_state = (uint8_t)g_status.mqtt_state;
    n.join_state = (uint8_t)g_status.join_state;
    for (i = 0U; i < 4U; i++) {
        n.ip[i] = g_status.dhcp_address[i];
    }
    n.init_result = sat16(g_status.init_result);
    n.join_result = sat16(g_status.join_result);
    n.rssi_dbm = sat16(g_status.link_rssi);
    n.strongest_dbm = sat16(g_status.strongest_rssi);
    n.networks_seen = (g_status.scan_count > 65535U) ? 65535U : (uint16_t)g_status.scan_count;
    n.join_attempts = (g_status.join_attempts > 65535U) ? 65535U : (uint16_t)g_status.join_attempts;
    n.join_failures = (g_status.join_failures > 65535U) ? 65535U : (uint16_t)g_status.join_failures;
    g_info = n;
}

void robot_net_get(robot_net_info_t *out)
{
    *out = g_info;
}

bool robot_net_link_up(void)
{
    return (g_info.link_up != 0U) && (g_info.dhcp != 0U);
}

uint32_t robot_net_mqtt_state(void)
{
    return (uint32_t)g_info.mqtt_state;
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

#else   /* no radio in this build */

void robot_net_refresh(void)
{
}

void robot_net_get(robot_net_info_t *out)
{
    uint8_t *p = (uint8_t *)out;
    uint32_t i;

    for (i = 0U; i < (uint32_t)sizeof(*out); i++) {     /* no <string.h>: it clashes with the kernel's size_t */
        p[i] = 0U;
    }
}

bool robot_net_link_up(void)
{
    return false;
}

uint32_t robot_net_mqtt_state(void)
{
    return 0U;
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
