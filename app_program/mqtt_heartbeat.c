#include "mqtt_heartbeat.h"

typedef struct {
    char *bytes;
    size_t capacity;
    size_t length;
    bool valid;
} heartbeat_writer_t;

static void write_text(heartbeat_writer_t *writer, const char *text)
{
    while (*text != '\0') {
        if (writer->length >= writer->capacity) {
            writer->valid = false;
            return;
        }
        writer->bytes[writer->length++] = *text++;
    }
}

static void write_u32(heartbeat_writer_t *writer, uint32_t value)
{
    char reversed[10];
    size_t count = 0U;

    do {
        reversed[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);
    while (count > 0U) {
        char digit[2] = { reversed[--count], '\0' };
        write_text(writer, digit);
    }
}

static void write_hex32(heartbeat_writer_t *writer, uint32_t value)
{
    static const char digits[] = "0123456789abcdef";
    unsigned int shift;

    for (shift = 32U; shift != 0U; shift -= 4U) {
        char digit[2] = {
            digits[(value >> (shift - 4U)) & 0x0fU], '\0'
        };
        write_text(writer, digit);
    }
}

void mqtt_heartbeat_init(mqtt_heartbeat_t *heartbeat, uint32_t boot_id)
{
    if (heartbeat == NULL) return;
    heartbeat->boot_id = boot_id;
    heartbeat->sequence = 0U;
    heartbeat->online_session = 0U;
    heartbeat->last_publish_ms = 0U;
    heartbeat->online = false;
}

void mqtt_heartbeat_set_session(mqtt_heartbeat_t *heartbeat, bool online,
                                uint32_t online_session, uint32_t now_ms)
{
    if (heartbeat == NULL) return;
    if (!online) {
        heartbeat->online = false;
        return;
    }
    if (!heartbeat->online || heartbeat->online_session != online_session) {
        heartbeat->online = true;
        heartbeat->online_session = online_session;
        heartbeat->last_publish_ms = now_ms;
    }
}

bool mqtt_heartbeat_due(const mqtt_heartbeat_t *heartbeat, uint32_t now_ms)
{
    return heartbeat != NULL && heartbeat->online &&
           (uint32_t)(now_ms - heartbeat->last_publish_ms) >=
               MQTT_HEARTBEAT_INTERVAL_MS;
}

size_t mqtt_heartbeat_prepare(const mqtt_heartbeat_t *heartbeat,
                              uint32_t now_ms, char *out, size_t capacity)
{
    heartbeat_writer_t writer = { out, capacity, 0U, true };

    if (heartbeat == NULL || out == NULL || capacity == 0U ||
        !mqtt_heartbeat_due(heartbeat, now_ms)) return 0U;
    write_text(&writer, "v=1,b=");
    write_hex32(&writer, heartbeat->boot_id);
    write_text(&writer, ",seq=");
    write_u32(&writer, heartbeat->sequence);
    write_text(&writer, ",up=");
    write_u32(&writer, now_ms);
    return writer.valid ? writer.length : 0U;
}

void mqtt_heartbeat_published(mqtt_heartbeat_t *heartbeat, uint32_t now_ms)
{
    if (heartbeat == NULL || !heartbeat->online) return;
    heartbeat->sequence++;
    heartbeat->last_publish_ms = now_ms;
}
