#include "mqtt_mailbox.h"

typedef struct {
    unsigned char topic_length;
    unsigned char payload_length;
    char topic[MQTT_MAILBOX_TOPIC_BYTES];
    unsigned char payload[MQTT_MAILBOX_PAYLOAD_BYTES];
} rx_slot_t;

static rx_slot_t g_rx[MQTT_MAILBOX_RX_SLOTS];
static volatile unsigned int g_rx_head;         /* next slot the owner writes: owner only */
static volatile unsigned int g_rx_tail;         /* next slot the robot reads: robot only */
static volatile unsigned int g_rx_dropped;

static char g_tx_topic[MQTT_MAILBOX_TOPIC_BYTES + 1U];
static unsigned char g_tx_payload[MQTT_MAILBOX_PAYLOAD_BYTES];
static unsigned int g_tx_length;
static volatile unsigned int g_tx_pending;      /* 0 empty, 1 filled (set last by the robot, cleared last by the owner) */

static void barrier(void)
{
    __sync_synchronize();
}

unsigned int mqtt_mailbox_rx_push(const unsigned char *topic, unsigned int topic_length,
                                  const unsigned char *payload, unsigned int payload_length)
{
    unsigned int head = g_rx_head;
    unsigned int i;
    rx_slot_t *slot;

    if ((topic == 0) || (topic_length == 0U) || (topic_length > MQTT_MAILBOX_TOPIC_BYTES) ||
        (payload_length > MQTT_MAILBOX_PAYLOAD_BYTES) || ((payload == 0) && (payload_length != 0U))) {
        g_rx_dropped++;
        return 0U;
    }
    if (((head + 1U) % MQTT_MAILBOX_RX_SLOTS) == g_rx_tail) {
        g_rx_dropped++;                         /* full: the newest message is the one dropped */
        return 0U;
    }
    slot = &g_rx[head];
    slot->topic_length = (unsigned char)topic_length;
    slot->payload_length = (unsigned char)payload_length;
    for (i = 0U; i < topic_length; i++) {
        slot->topic[i] = (char)topic[i];
    }
    for (i = 0U; i < payload_length; i++) {
        slot->payload[i] = payload[i];
    }
    barrier();                                  /* the slot is complete before the robot can see it */
    g_rx_head = (head + 1U) % MQTT_MAILBOX_RX_SLOTS;
    return 1U;
}

unsigned int mqtt_mailbox_rx_pop(char *topic, unsigned char *payload, unsigned int *payload_length)
{
    unsigned int tail = g_rx_tail;
    unsigned int i;
    const rx_slot_t *slot;

    if (tail == g_rx_head) {
        return 0U;
    }
    barrier();
    slot = &g_rx[tail];
    for (i = 0U; i < slot->topic_length; i++) {
        topic[i] = slot->topic[i];
    }
    topic[slot->topic_length] = '\0';
    for (i = 0U; i < slot->payload_length; i++) {
        payload[i] = slot->payload[i];
    }
    *payload_length = slot->payload_length;
    barrier();                                  /* done reading before the owner may reuse the slot */
    g_rx_tail = (tail + 1U) % MQTT_MAILBOX_RX_SLOTS;
    return 1U;
}

unsigned int mqtt_mailbox_rx_dropped(void)
{
    return g_rx_dropped;
}

unsigned int mqtt_mailbox_tx_put(const char *topic, const unsigned char *payload, unsigned int payload_length)
{
    unsigned int i = 0U;

    if ((g_tx_pending != 0U) || (topic == 0) || (payload_length > MQTT_MAILBOX_PAYLOAD_BYTES) ||
        ((payload == 0) && (payload_length != 0U))) {
        return 0U;
    }
    while ((topic[i] != '\0') && (i < MQTT_MAILBOX_TOPIC_BYTES)) {
        g_tx_topic[i] = topic[i];
        i++;
    }
    if ((i == 0U) || (topic[i] != '\0')) {
        return 0U;                              /* empty or too long */
    }
    g_tx_topic[i] = '\0';
    for (i = 0U; i < payload_length; i++) {
        g_tx_payload[i] = payload[i];
    }
    g_tx_length = payload_length;
    barrier();
    g_tx_pending = 1U;
    return 1U;
}

unsigned int mqtt_mailbox_tx_peek(char *topic, unsigned char *payload, unsigned int *payload_length)
{
    unsigned int i;

    if (g_tx_pending == 0U) {
        return 0U;
    }
    barrier();
    for (i = 0U; i <= MQTT_MAILBOX_TOPIC_BYTES; i++) {
        topic[i] = g_tx_topic[i];
        if (g_tx_topic[i] == '\0') {
            break;
        }
    }
    for (i = 0U; i < g_tx_length; i++) {
        payload[i] = g_tx_payload[i];
    }
    *payload_length = g_tx_length;
    return 1U;
}

void mqtt_mailbox_tx_done(void)
{
    barrier();
    g_tx_pending = 0U;
}

void mqtt_mailbox_reset(void)
{
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_rx_dropped = 0U;
    g_tx_pending = 0U;
    g_tx_length = 0U;
}
