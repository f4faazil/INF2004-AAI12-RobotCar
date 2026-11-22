#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mqtt_heartbeat.h"

int main(void)
{
    mqtt_heartbeat_t heartbeat;
    uint8_t payload[MQTT_HEARTBEAT_PAYLOAD_BYTES];
    size_t length;

    mqtt_heartbeat_init(&heartbeat, UINT32_C(0x12abcdef));
    assert(!mqtt_heartbeat_due(&heartbeat, 10000U));

    mqtt_heartbeat_set_session(&heartbeat, true, 1U, 100U);
    assert(!mqtt_heartbeat_due(&heartbeat, 10099U));
    assert(mqtt_heartbeat_due(&heartbeat, 10100U));
    length = mqtt_heartbeat_prepare(&heartbeat, 10100U, payload,
                                    sizeof payload);
    /* 13 bytes: type 0x21, boot id, sequence 0, uptime 10100 (0x2774), all big-endian. */
    {
        static const uint8_t expected[13] = {
            0x21, 0x12, 0xab, 0xcd, 0xef, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x27, 0x74
        };
        assert(length == sizeof expected);
        assert(memcmp(payload, expected, sizeof expected) == 0);
    }
    /* A buffer that is too small gives nothing, not a cut-off record. */
    assert(mqtt_heartbeat_prepare(&heartbeat, 10100U, payload, 12U) == 0U);

    /* Preparing or a failed transport does not consume a sequence. */
    assert(mqtt_heartbeat_prepare(&heartbeat, 10101U, payload,
                                  sizeof payload) == length);
    mqtt_heartbeat_published(&heartbeat, 10101U);
    assert(heartbeat.sequence == 1U);
    assert(!mqtt_heartbeat_due(&heartbeat, 20100U));
    assert(mqtt_heartbeat_due(&heartbeat, 20101U));

    /* Disconnect suppresses publication. Reconnect preserves boot sequence
     * and starts a fresh interval rather than bursting immediately. */
    mqtt_heartbeat_set_session(&heartbeat, false, 1U, 20101U);
    assert(!mqtt_heartbeat_due(&heartbeat, 40000U));
    mqtt_heartbeat_set_session(&heartbeat, true, 2U, 40000U);
    assert(!mqtt_heartbeat_due(&heartbeat, 49999U));
    assert(mqtt_heartbeat_due(&heartbeat, 50000U));
    length = mqtt_heartbeat_prepare(&heartbeat, 50000U, payload,
                                    sizeof payload);
    assert(length == 13U);
    assert(payload[8] == 1U);                              /* sequence is now 1 */
    assert(payload[9] == 0x00 && payload[10] == 0x00 &&
           payload[11] == 0xc3 && payload[12] == 0x50);    /* uptime 50000 = 0xc350 */

    /* Unsigned elapsed arithmetic remains correct across sys_now rollover. */
    mqtt_heartbeat_published(&heartbeat, UINT32_MAX - 4999U);
    assert(!mqtt_heartbeat_due(&heartbeat, 4999U));
    assert(mqtt_heartbeat_due(&heartbeat, 5000U));

    puts("PASS: MQTT heartbeat schedule/sequence/reconnect/rollover tests");
    return 0;
}
