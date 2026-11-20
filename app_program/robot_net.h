/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * robot_net.h
 * The robot's view of the network: is the WiFi link up, and a byte-stream
 * transport for the MQTT client.  Two implementations share this interface:
 *   - the default build has no radio (the qualified profile keeps the CYW43439
 *     powered down), so the link is reported down and nothing connects;
 *   - a WIFI=cyw43 WIFI_MQTT=1 build uses the lwIP raw-TCP link that runs
 *     inside the radio owner task (lib/libnet/lwip/lwip_utk_link.c).
 * Either way the control tasks are unaffected: telemetry simply drops.
 */
#ifndef ROBOT_NET_H
#define ROBOT_NET_H

#include <stdbool.h>
#include <stdint.h>

#include "mqtt_client.h"

/*
 * The radio's status block (T_CYW43_UTK_STATUS) is 2576 bytes: far too big for the 2 KB stack of a control task, and
 * copying it there overflowed the Vehicle task's stack and stalled the whole core.  So ONE task (net_task, which has a big
 * stack) reads it, in robot_net_refresh(), and everything else asks for this small cached copy.
 */
typedef struct {
    uint8_t  ready;             /* the radio chip answered */
    uint8_t  link_up;           /* joined a network */
    uint8_t  dhcp;              /* has an IP address */
    uint8_t  mqtt_state;        /* 0 wait-net, 1 tcp-connect, 2 wait-ack, 3 ONLINE, 4 backoff */
    uint8_t  join_state;        /* CYW43_UTK_JOIN_* */
    uint8_t  ip[4];
    int16_t  init_result;
    int16_t  join_result;
    int16_t  rssi_dbm;
    int16_t  strongest_dbm;
    uint16_t networks_seen;
    uint16_t join_attempts;
    uint16_t join_failures;
} robot_net_info_t;

/* Net task only: read the radio's status into the cache. */
void robot_net_refresh(void);

/* Any task: a copy of the cache. */
void robot_net_get(robot_net_info_t *out);

/* True when the WiFi link is up and an IP address has been obtained. */
bool robot_net_link_up(void);

/* The MQTT session state owned by the radio task (mqtt_lwip.c): 0 wait-net, 1 tcp-connect, 2 wait-ack,
 * 3 ONLINE, 4 backoff.  Always 0 in a build without the radio. */
uint32_t robot_net_mqtt_state(void);

/* The transport to give the MQTT client. */
mqtt_transport_t robot_net_transport(void);

#endif /* ROBOT_NET_H */
