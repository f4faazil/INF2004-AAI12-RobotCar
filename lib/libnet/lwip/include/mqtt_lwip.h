#ifndef AAI12_MQTT_LWIP_H
#define AAI12_MQTT_LWIP_H

struct netif;

/* Called ONLY by the existing CYW43/lwIP owner task, never from another
 * T-Kernel task, a timer interrupt, or the second core. */
void mqtt_lwip_poll(struct netif *netif);

#endif /* AAI12_MQTT_LWIP_H */
