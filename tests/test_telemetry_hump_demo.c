#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "telemetry_hump_demo.h"

static void test_known_frame(void)
{
    uint8_t bytes[AAI12_HUMP_DEMO_BYTES] = {0};
    uint8_t status = 0U, height = 0U;
    bool invalid = true, fault = true;
    const uint8_t expected[3] = {0x04U, 0x25U, 0x03U};

    assert(telemetry_hump_demo_encode(bytes, 5U, 100U, false, false));
    assert(memcmp(bytes, expected, sizeof expected) == 0);
    assert(telemetry_hump_demo_decode(bytes, sizeof bytes,
                                      &status, &height, &invalid, &fault));
    assert(status == 5U && height == 100U && !invalid && !fault);
}

static void test_out_of_range_marker(void)
{
    uint8_t bytes[3];
    uint8_t status = 0U, height = 0U;
    bool invalid = false, fault = false;

    assert(telemetry_hump_demo_encode(bytes, 1U, 200U, true, true));
    assert(telemetry_hump_demo_decode(bytes, sizeof bytes,
                                      &status, &height, &invalid, &fault));
    assert(status == 1U && height == 127U && invalid && fault);
    assert(!telemetry_hump_demo_encode(bytes, 1U, 127U, false, false));
}

static void test_reject_invalid_without_output_mutation(void)
{
    uint8_t bytes[3] = {0xa5U, 0xa5U, 0xa5U};
    const uint8_t before[3] = {0xa5U, 0xa5U, 0xa5U};
    uint8_t status = 99U, height = 99U;
    bool invalid = false, fault = false;

    assert(!telemetry_hump_demo_encode(bytes, 8U, 100U, false, false));
    assert(memcmp(bytes, before, sizeof bytes) == 0);
    assert(!telemetry_hump_demo_encode(NULL, 1U, 20U, false, false));
    assert(telemetry_hump_demo_encode(bytes, 5U, 100U, false, false));
    bytes[2] |= 0x80U; /* reserved padding */
    assert(!telemetry_hump_demo_decode(bytes, sizeof bytes,
                                       &status, &height, &invalid, &fault));
    assert(status == 99U && height == 99U);
}

static void test_malformed_values(void)
{
    uint8_t bytes[3];
    uint8_t status = 0U, height = 0U;
    bool invalid = false, fault = false;

    assert(telemetry_hump_demo_encode(bytes, 2U, 23U, false, true));
    bytes[0] = 0x05U;
    assert(!telemetry_hump_demo_decode(bytes, sizeof bytes,
                                       &status, &height, &invalid, &fault));
    bytes[0] = 0x04U;
    assert(!telemetry_hump_demo_decode(bytes, 2U,
                                       &status, &height, &invalid, &fault));
    assert(!telemetry_hump_demo_decode(NULL, 3U,
                                       &status, &height, &invalid, &fault));
}

int main(void)
{
    test_known_frame();
    test_out_of_range_marker();
    test_reject_invalid_without_output_mutation();
    test_malformed_values();
    puts("PASS: 4 diagnostic 13-bit hump frame tests");
    return 0;
}
