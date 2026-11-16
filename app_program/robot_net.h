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

#include "mqtt_client.h"

/* True when the WiFi link is up and an IP address has been obtained. */
bool robot_net_link_up(void);

/* The transport to give the MQTT client. */
mqtt_transport_t robot_net_transport(void);

#endif /* ROBOT_NET_H */
