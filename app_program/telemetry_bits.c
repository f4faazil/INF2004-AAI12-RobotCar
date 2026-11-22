#include "telemetry_bits.h"

#include <limits.h>
#include <string.h>

static bool valid_bit_length(size_t bytes)
{
    return bytes <= SIZE_MAX / 8U;
}

bool telemetry_bits_writer_init(telemetry_bit_writer_t *writer,
                                uint8_t *data, size_t capacity_bytes)
{
    if (writer == NULL || data == NULL || !valid_bit_length(capacity_bytes)) {
        return false;
    }

    memset(data, 0, capacity_bytes);
    writer->data = data;
    writer->capacity_bytes = capacity_bytes;
    writer->used_bits = 0U;
    return true;
}

bool telemetry_bits_write(telemetry_bit_writer_t *writer,
                          uint32_t value, unsigned width_bits)
{
    if (writer == NULL || writer->data == NULL || width_bits == 0U ||
        width_bits > 32U || !valid_bit_length(writer->capacity_bytes) ||
        writer->used_bits > writer->capacity_bytes * 8U) {
        return false;
    }

    /* Reject out-of-range fields instead of silently truncating. */
    if ((uint64_t)value >= (UINT64_C(1) << width_bits)) {
        return false;
    }

    if ((size_t)width_bits > writer->capacity_bytes * 8U - writer->used_bits) {
        return false;
    }

    for (unsigned i = 0U; i < width_bits; ++i) {
        const size_t bit_index = writer->used_bits + i;
        const size_t byte_index = bit_index / 8U;
        const uint8_t mask = (uint8_t)(1U << (bit_index % 8U));
        if (((value >> i) & 1U) != 0U) {
            writer->data[byte_index] |= mask;
        } else {
            writer->data[byte_index] &= (uint8_t)(~mask);
        }
    }

    writer->used_bits += width_bits;
    return true;
}

size_t telemetry_bits_written_bytes(const telemetry_bit_writer_t *writer)
{
    if (writer == NULL) {
        return 0U;
    }
    return writer->used_bits / 8U + ((writer->used_bits % 8U) != 0U ? 1U : 0U);
}

bool telemetry_bits_reader_init(telemetry_bit_reader_t *reader,
                                const uint8_t *data, size_t length_bytes)
{
    if (reader == NULL || data == NULL || !valid_bit_length(length_bytes)) {
        return false;
    }
    reader->data = data;
    reader->length_bytes = length_bytes;
    reader->used_bits = 0U;
    return true;
}

bool telemetry_bits_read(telemetry_bit_reader_t *reader,
                         unsigned width_bits, uint32_t *value)
{
    if (reader == NULL || reader->data == NULL || value == NULL ||
        width_bits == 0U || width_bits > 32U ||
        !valid_bit_length(reader->length_bytes) ||
        reader->used_bits > reader->length_bytes * 8U) {
        return false;
    }
    if ((size_t)width_bits > reader->length_bytes * 8U - reader->used_bits) {
        return false;
    }

    uint32_t result = 0U;
    for (unsigned i = 0U; i < width_bits; ++i) {
        const size_t bit_index = reader->used_bits + i;
        const uint32_t bit = (uint32_t)((reader->data[bit_index / 8U]
                                       >> (bit_index % 8U)) & 1U);
        result |= bit << i;
    }
    reader->used_bits += width_bits;
    *value = result;
    return true;
}
