#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../app_program/mqtt_mailbox.h"

static const unsigned char *u(const char *s)
{
    return (const unsigned char *)s;
}

static void test_rx_fifo_order_and_content(void)
{
    char topic[MQTT_MAILBOX_TOPIC_BYTES + 1U];
    unsigned char payload[MQTT_MAILBOX_PAYLOAD_BYTES];
    unsigned int n = 0U;

    mqtt_mailbox_reset();
    assert(mqtt_mailbox_rx_pop(topic, payload, &n) == 0U);
    assert(mqtt_mailbox_rx_push(u("aai12/robot/map"), 15U, u("A=L"), 3U) == 1U);
    assert(mqtt_mailbox_rx_push(u("aai12/robot/cmd"), 15U, u("START"), 5U) == 1U);
    assert(mqtt_mailbox_rx_pop(topic, payload, &n) == 1U);
    assert(strcmp(topic, "aai12/robot/map") == 0 && n == 3U && memcmp(payload, "A=L", 3U) == 0);
    assert(mqtt_mailbox_rx_pop(topic, payload, &n) == 1U);
    assert(strcmp(topic, "aai12/robot/cmd") == 0 && n == 5U && memcmp(payload, "START", 5U) == 0);
    assert(mqtt_mailbox_rx_pop(topic, payload, &n) == 0U);
}

/* A full ring drops the NEWEST message and counts it; older ones survive; long input is refused, not truncated. */
static void test_rx_full_and_oversize(void)
{
    char topic[MQTT_MAILBOX_TOPIC_BYTES + 1U];
    unsigned char payload[MQTT_MAILBOX_PAYLOAD_BYTES];
    unsigned char big[MQTT_MAILBOX_PAYLOAD_BYTES + 1U];
    unsigned int n = 0U;
    unsigned int i;

    mqtt_mailbox_reset();
    memset(big, 'x', sizeof big);
    assert(mqtt_mailbox_rx_push(u("t"), 1U, big, sizeof big) == 0U);
    assert(mqtt_mailbox_rx_push(u("t"), 0U, big, 1U) == 0U);
    assert(mqtt_mailbox_rx_push(u("0123456789012345678901234567890123"), 34U, big, 1U) == 0U);
    assert(mqtt_mailbox_rx_dropped() == 3U);
    assert(mqtt_mailbox_rx_push(u("t"), 1U, big, MQTT_MAILBOX_PAYLOAD_BYTES) == 1U);   /* exactly the full size */
    for (i = 1U; i < MQTT_MAILBOX_RX_SLOTS - 1U; i++) {
        unsigned char b = (unsigned char)('0' + i);

        assert(mqtt_mailbox_rx_push(u("t"), 1U, &b, 1U) == 1U);
    }
    assert(mqtt_mailbox_rx_push(u("t"), 1U, u("Z"), 1U) == 0U);                        /* full */
    assert(mqtt_mailbox_rx_dropped() == 4U);
    assert(mqtt_mailbox_rx_pop(topic, payload, &n) == 1U && n == MQTT_MAILBOX_PAYLOAD_BYTES);
    assert(mqtt_mailbox_rx_push(u("t"), 1U, u("Z"), 1U) == 1U);                        /* room again */
    for (i = 0U; i < MQTT_MAILBOX_RX_SLOTS - 1U; i++) {
        assert(mqtt_mailbox_rx_pop(topic, payload, &n) == 1U);
    }
    assert(payload[0] == 'Z' && mqtt_mailbox_rx_pop(topic, payload, &n) == 0U);
}

/* The ring wraps many times without losing or reordering anything. */
static void test_rx_wraparound(void)
{
    char topic[MQTT_MAILBOX_TOPIC_BYTES + 1U];
    unsigned char payload[MQTT_MAILBOX_PAYLOAD_BYTES];
    unsigned int n = 0U;
    unsigned int i;

    mqtt_mailbox_reset();
    for (i = 0U; i < 100U; i++) {
        unsigned char b = (unsigned char)i;

        assert(mqtt_mailbox_rx_push(u("t"), 1U, &b, 1U) == 1U);
        assert(mqtt_mailbox_rx_pop(topic, payload, &n) == 1U && n == 1U && payload[0] == b);
    }
    assert(mqtt_mailbox_rx_dropped() == 0U);
}

/* One outgoing message at a time: refused while the owner has not sent the last; repeatable until marked done. */
static void test_tx_single_slot(void)
{
    char topic[MQTT_MAILBOX_TOPIC_BYTES + 1U];
    unsigned char payload[MQTT_MAILBOX_PAYLOAD_BYTES];
    unsigned int n = 0U;

    mqtt_mailbox_reset();
    assert(mqtt_mailbox_tx_peek(topic, payload, &n) == 0U);
    assert(mqtt_mailbox_tx_put("aai12/robot/map_ack", u("OK n=26"), 7U) == 1U);
    assert(mqtt_mailbox_tx_put("aai12/robot/map_ack", u("ERR"), 3U) == 0U);            /* busy */
    assert(mqtt_mailbox_tx_peek(topic, payload, &n) == 1U);
    assert(strcmp(topic, "aai12/robot/map_ack") == 0 && n == 7U && memcmp(payload, "OK n=26", 7U) == 0);
    assert(mqtt_mailbox_tx_peek(topic, payload, &n) == 1U);                            /* still there until done */
    mqtt_mailbox_tx_done();
    assert(mqtt_mailbox_tx_peek(topic, payload, &n) == 0U);
    assert(mqtt_mailbox_tx_put("aai12/robot/map_ack", u("ERR"), 3U) == 1U);
    assert(mqtt_mailbox_tx_peek(topic, payload, &n) == 1U && n == 3U && memcmp(payload, "ERR", 3U) == 0);
    mqtt_mailbox_tx_done();
    assert(mqtt_mailbox_tx_put("", u("x"), 1U) == 0U);
    assert(mqtt_mailbox_tx_put("0123456789012345678901234567890123", u("x"), 1U) == 0U);
}

int main(void)
{
    test_rx_fifo_order_and_content();
    test_rx_full_and_oversize();
    test_rx_wraparound();
    test_tx_single_slot();
    puts("PASS: 4 MQTT mailbox tests");
    return 0;
}
