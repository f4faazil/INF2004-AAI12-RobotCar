/*
 * Broker endpoint for the robot's MQTT telemetry (WIFI_MQTT=1 build).
 *
 * Copy this file to config/mqtt_config.h and set the address of the machine
 * running the MQTT broker (for example Mosquitto on the team laptop, port
 * 1883).  The local copy is ignored by Git because addresses vary between
 * networks.  The robot connects with an IPv4 address, not a host name.
 */
#ifndef UTK3_MQTT_CONFIG_H
#define UTK3_MQTT_CONFIG_H

/*
 * Copy this file to config/mqtt_config.h, fill in the values below, then change
 * the 0 on the next line to 1.  It is a value to flip rather than a line to
 * delete, so an unedited copy fails loudly at compile time instead of building
 * fine and never connecting.
 */
#define UTK_MQTT_CONFIG_SET   0

#if !UTK_MQTT_CONFIG_SET
#error "config/mqtt_config.h: placeholder values. Set the broker address, then set UTK_MQTT_CONFIG_SET to 1."
#endif

#define UTK_MQTT_BROKER_ADDRESS "192.168.1.100"
#define UTK_MQTT_BROKER_PORT    1883U

#endif
