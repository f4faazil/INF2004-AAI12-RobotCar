/*
 * Buddy 1 opt-in MQTT smoke-test configuration.
 * Copy this file to config/mqtt_test_config.h before WIFI_MQTT=1 builds.
 * This probe is plaintext, unauthenticated QoS 0 on your trusted hotspot.
 * Do not deploy it unchanged beyond the controlled development network.
 */
#ifndef AAI12_MQTT_TEST_CONFIG_H
#define AAI12_MQTT_TEST_CONFIG_H

#define AAI12_MQTT_BROKER_IP   "172.20.10.3"
#define AAI12_MQTT_BROKER_PORT 1883U
#define AAI12_MQTT_CLIENT_ID   "aai12-picow-dev"
#define AAI12_MQTT_TEST_TOPIC  "aai12/test"

#endif /* AAI12_MQTT_TEST_CONFIG_H */
