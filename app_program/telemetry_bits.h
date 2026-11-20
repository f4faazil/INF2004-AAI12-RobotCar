#ifndef AAI12_TELEMETRY_BITS_H
#define AAI12_TELEMETRY_BITS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Portable bit-packing utility for AAI12 binary telemetry.
 * Wire order is defined explicitly:
 *   - fields are written in schema order;
 *   - each integer's least-significant bit is transmitted first;
 *   - bits fill byte bit 0 through bit 7 before the next byte.
 * Do not use C struct bit-fields as the MQTT wire representation.
 *
 * This is NOT the final telemetry schema. Field definitions require
 * confirmation from the post-Tribunal decisions and subsystem owners.
 */
typedef struct {
    uint8_t *data;
    size_t capacity_bytes;
    size_t used_bits;
} telemetry_bit_writer_t;

typedef struct {
    const uint8_t *data;
    size_t length_bytes;
    size_t used_bits;
} telemetry_bit_reader_t;

bool telemetry_bits_writer_init(telemetry_bit_writer_t *writer,
                                uint8_t *data, size_t capacity_bytes);

/* Atomic on failure at the function level: no position/data change. */
bool telemetry_bits_write(telemetry_bit_writer_t *writer,
                          uint32_t value, unsigned width_bits);

size_t telemetry_bits_written_bytes(const telemetry_bit_writer_t *writer);

bool telemetry_bits_reader_init(telemetry_bit_reader_t *reader,
                                const uint8_t *data, size_t length_bytes);

/* Atomic on failure at the function level: no position/output change. */
bool telemetry_bits_read(telemetry_bit_reader_t *reader,
                         unsigned width_bits, uint32_t *value);

#endif /* AAI12_TELEMETRY_BITS_H */
