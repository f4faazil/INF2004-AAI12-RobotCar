/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ultrasonic.h
 * HC-SR04 measurement handling (Buddy 5): echo pulse width to distance, a
 * small median filter, and the forward-obstacle monitor.
 *
 * The rules from the Tribunal are built in: "no echo" is never read as free
 * space (it is an invalid reading), too-close readings are flagged rather
 * than trusted, and a run of invalid readings becomes a sensor fault.
 *
 * Pure C: the HAL times the ECHO pin with interrupt capture and passes the
 * pulse width in microseconds.
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

#define US_MIN_RANGE_MM         (20U)       /* HC-SR04: 2 cm .. 4 m */
#define US_MAX_RANGE_MM         (4000U)
#define US_MEDIAN_LEN           (3U)

typedef enum {
    US_VALID = 0,
    US_NO_ECHO,         /* timed out: NOT evidence of free space */
    US_TOO_CLOSE,       /* inside the sensor's blind zone */
    US_OUT_OF_RANGE,    /* beyond the sensor's range */
    US_ERROR            /* hardware/timing fault */
} us_status_t;

typedef struct {
    us_status_t status;
    uint16_t    distance_mm;    /* meaningful only when status == US_VALID */
    uint32_t    timestamp_ms;
    uint32_t    seq;            /* increments with every completed measurement */
} us_reading_t;

/*
 * Convert one echo.  echo_complete is false when the echo never ended (a
 * timeout).  Speed of sound is taken as 343 m/s (20 C); the round trip is
 * halved.  *distance_mm is set only for US_VALID.
 */
us_status_t us_convert(uint32_t echo_us, bool echo_complete, uint16_t *distance_mm);

/* ---- filter and forward monitor -------------------------------------- */

typedef struct {
    uint16_t window[US_MEDIAN_LEN];
    uint8_t  count;
    uint8_t  head;
    uint8_t  invalid_run;           /* consecutive invalid readings */
    uint8_t  invalid_limit;         /* this many in a row is a sensor fault */
    uint8_t  confirm_count;         /* readings needed to flip obstacle state */
    uint8_t  below_run;
    uint8_t  above_run;
    bool     obstacle;
    uint16_t last_median_mm;
    bool     have_median;
    uint32_t last_seq;              /* a repeated sequence number is one measurement */
    bool     have_seq;
} us_monitor_t;

void us_monitor_init(us_monitor_t *m, uint8_t confirm_count, uint8_t invalid_limit);

/*
 * Feed one reading.  Obstacle is declared after confirm_count consecutive
 * valid medians below threshold_mm and cleared after confirm_count above
 * threshold_mm + hysteresis_mm.  Invalid readings neither declare nor clear
 * an obstacle.  Returns the obstacle state.
 *
 * urgent_mm (0 = off): a single VALID reading closer than this declares the
 * obstacle immediately, bypassing the median and the confirmation count.  The
 * normal path costs two measurement cycles (140 ms at 70 ms per measurement),
 * which would not fit the 150 ms obstacle-to-stop budget for something
 * already very close.
 */
bool us_monitor_update(us_monitor_t *m, const us_reading_t *reading,
                       uint16_t threshold_mm, uint16_t hysteresis_mm,
                       uint16_t urgent_mm);

/* True when invalid readings have persisted: report FAULT_ULTRASONIC. */
bool us_monitor_sensor_fault(const us_monitor_t *m);

/* The latest filtered distance, or false when none is available. */
bool us_monitor_distance(const us_monitor_t *m, uint16_t *distance_mm);

/*
 * Detection range needed to stop in time: distance covered during the
 * response time, plus the braking distance and a margin.  The braking
 * distance must be measured on the car; the default is a placeholder.
 */
uint16_t us_stop_threshold_mm(int16_t speed_mm_s, uint16_t response_ms,
                              uint16_t braking_mm, uint16_t margin_mm);

#endif /* ULTRASONIC_H */
