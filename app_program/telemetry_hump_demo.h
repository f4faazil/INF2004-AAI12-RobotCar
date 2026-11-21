#ifndef AAI12_TELEMETRY_HUMP_DEMO_H
#define AAI12_TELEMETRY_HUMP_DEMO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* DIAGNOSTIC FORMAT ONLY, not the team's approved final wire schema.
 * A single 8-bit example type followed by a 13-bit example hump body.
 * Bit order is LSB-first, as implemented in telemetry_bits.c.
 * Field order: type(8), status(3), height(7), validity(2), fault(1).
 * Validity: 0=valid, 3=out of range. Height 127 must use validity=3.
 */
#define AAI12_HUMP_DEMO_BYTES 3U

/* For normal samples pass a height 0..126 and out_of_range=false.
 * For invalid samples pass out_of_range=true; height is encoded as 127.
 * Writes exactly 3 bytes or returns false without exposing a partial result.
 */
bool telemetry_hump_demo_encode(uint8_t out[AAI12_HUMP_DEMO_BYTES],
                                uint8_t status, uint8_t height,
                                bool out_of_range, bool imu_fault);

/* Strict diagnostic decoder, useful for host tests and message inspection. */
bool telemetry_hump_demo_decode(const uint8_t *data, size_t length,
                                uint8_t *status, uint8_t *height,
                                bool *out_of_range, bool *imu_fault);

#endif /* AAI12_TELEMETRY_HUMP_DEMO_H */
