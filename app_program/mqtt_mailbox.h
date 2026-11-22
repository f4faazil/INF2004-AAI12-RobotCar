#ifndef AAI12_MQTT_MAILBOX_H
#define AAI12_MQTT_MAILBOX_H

/*
 * Two small mailboxes between the MQTT owner task (the radio task, which owns the session) and the robot's own tasks.
 *
 *   receive:  broker -> owner -> robot.   The owner pushes each incoming message (a letter map, a command); the robot's
 *             network task pops it.  A single-producer, single-consumer ring: no lock, safe across the two cores.
 *   send:     robot -> owner -> broker.   One pending outgoing message (the map acknowledgement).  The owner peeks it,
 *             publishes it, and then marks it done; the robot can put a new one only when the slot is empty.
 *
 * No <stddef.h> and no <string.h>: this header is included next to the micro T-Kernel headers, whose size_t does not
 * match the C library's.
 */

#define MQTT_MAILBOX_TOPIC_BYTES    32U     /* longest topic stored, without the terminator */
#define MQTT_MAILBOX_PAYLOAD_BYTES  32U     /* longest payload stored: a 26-letter binary map is 27 bytes, a command 5. Longer messages are dropped and counted
                                         * (was 128, sized for the text map; a text map of more than 32 bytes is no longer accepted) */
#define MQTT_MAILBOX_RX_SLOTS       4U

/* Owner task: store an incoming message.  1 = stored; 0 = dropped (full, or too long): counted. */
unsigned int mqtt_mailbox_rx_push(const unsigned char *topic, unsigned int topic_length,
                                  const unsigned char *payload, unsigned int payload_length);

/* Robot task: take the oldest message.  topic gets a terminator (MQTT_MAILBOX_TOPIC_BYTES + 1 bytes), payload up to
 * MQTT_MAILBOX_PAYLOAD_BYTES.  1 = a message was returned. */
unsigned int mqtt_mailbox_rx_pop(char *topic, unsigned char *payload, unsigned int *payload_length);

unsigned int mqtt_mailbox_rx_dropped(void);

/* Robot task: queue one outgoing message.  1 = accepted; 0 = the slot is busy (try again later) or the message is too long. */
unsigned int mqtt_mailbox_tx_put(const char *topic, const unsigned char *payload, unsigned int payload_length);

/* Owner task: copy the pending message without removing it.  1 = there is one. */
unsigned int mqtt_mailbox_tx_peek(char *topic, unsigned char *payload, unsigned int *payload_length);

/* Owner task: the peeked message was handed to the transport. */
void mqtt_mailbox_tx_done(void);

/* For tests and a reconnect: forget everything. */
void mqtt_mailbox_reset(void);

#endif /* AAI12_MQTT_MAILBOX_H */
