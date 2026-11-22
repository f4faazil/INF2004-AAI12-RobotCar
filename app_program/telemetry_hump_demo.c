#include "telemetry_hump_demo.h"
#include "telemetry_bits.h"

#include <string.h>

#define AAI12_HUMP_DEMO_TYPE 0x04U

bool telemetry_hump_demo_encode(uint8_t out[AAI12_HUMP_DEMO_BYTES],
                                uint8_t status, uint8_t height,
                                bool out_of_range, bool imu_fault)
{
    uint8_t temporary[AAI12_HUMP_DEMO_BYTES];
    telemetry_bit_writer_t writer;

    if (out == NULL || status > 7U || (!out_of_range && height > 126U)) {
        return false;
    }
    if (!telemetry_bits_writer_init(&writer, temporary, sizeof temporary) ||
        !telemetry_bits_write(&writer, AAI12_HUMP_DEMO_TYPE, 8U) ||
        !telemetry_bits_write(&writer, status, 3U) ||
        !telemetry_bits_write(&writer, out_of_range ? 127U : height, 7U) ||
        !telemetry_bits_write(&writer, out_of_range ? 3U : 0U, 2U) ||
        !telemetry_bits_write(&writer, imu_fault ? 1U : 0U, 1U) ||
        telemetry_bits_written_bytes(&writer) != sizeof temporary) {
        return false;
    }
    memcpy(out, temporary, sizeof temporary);
    return true;
}

bool telemetry_hump_demo_decode(const uint8_t *data, size_t length,
                                uint8_t *status, uint8_t *height,
                                bool *out_of_range, bool *imu_fault)
{
    telemetry_bit_reader_t reader;
    uint32_t type, raw_status, raw_height, validity, raw_fault;

    if (data == NULL || length != AAI12_HUMP_DEMO_BYTES || status == NULL ||
        height == NULL || out_of_range == NULL || imu_fault == NULL ||
        (data[2] & 0xe0U) != 0U ||
        !telemetry_bits_reader_init(&reader, data, length) ||
        !telemetry_bits_read(&reader, 8U, &type) ||
        !telemetry_bits_read(&reader, 3U, &raw_status) ||
        !telemetry_bits_read(&reader, 7U, &raw_height) ||
        !telemetry_bits_read(&reader, 2U, &validity) ||
        !telemetry_bits_read(&reader, 1U, &raw_fault)) {
        return false;
    }
    if (type != AAI12_HUMP_DEMO_TYPE ||
        !((validity == 0U && raw_height <= 126U) ||
          (validity == 3U && raw_height == 127U))) {
        return false;
    }
    *status = (uint8_t)raw_status;
    *height = (uint8_t)raw_height;
    *out_of_range = (validity == 3U);
    *imu_fault = (raw_fault != 0U);
    return true;
}
