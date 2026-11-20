#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mqtt_heartbeat.h"

int main(void)
{
    mqtt_heartbeat_t heartbeat;
    char payload[MQTT_HEARTBEAT_PAYLOAD_BYTES];
    size_t length;

    mqtt_heartbeat_init(&heartbeat, UINT32_C(0x12abcdef));
    assert(!mqtt_heartbeat_due(&heartbeat, 10000U));

    mqtt_heartbeat_set_session(&heartbeat, true, 1U, 100U);
    assert(!mqtt_heartbeat_due(&heartbeat, 10099U));
    assert(mqtt_heartbeat_due(&heartbeat, 10100U));
    length = mqtt_heartbeat_prepare(&heartbeat, 10100U, payload,
                                    sizeof payload);
    assert(length > 0U);
    assert(length < sizeof payload);
    payload[length] = '\0';
    assert(strcmp(payload, "v=1,b=12abcdef,seq=0,up=10100") == 0);

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
    payload[length] = '\0';
    assert(strstr(payload, "seq=1") != NULL);

    /* Unsigned elapsed arithmetic remains correct across sys_now rollover. */
    mqtt_heartbeat_published(&heartbeat, UINT32_MAX - 4999U);
    assert(!mqtt_heartbeat_due(&heartbeat, 4999U));
    assert(mqtt_heartbeat_due(&heartbeat, 5000U));

    puts("PASS: MQTT heartbeat schedule/sequence/reconnect/rollover tests");
    return 0;
}
