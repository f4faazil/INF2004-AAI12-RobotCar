#ifndef B5_CONFIG_H
#define B5_CONFIG_H
/* Proposed values: bench validate before enabling vehicle movement. */
#define B5_TRIG_GP 4u
#define B5_ECHO_GP 5u
#define B5_SERVO_GP 12u
/* FIRST set centre facing forward mechanically and measure endpoints.
 * Defaults deliberately inhibit scans; do not mark calibrated just to bypass it.
 * Logical 30 deg = robot right, 90 = forward, 150 = robot left. */
#define B5_SERVO_CALIBRATED 1
#define B5_SERVO_RIGHT_US 1100u
#define B5_SERVO_CENTRE_US 1500u
#define B5_SERVO_LEFT_US 1850u
#define B5_SERVO_SETTLE_MS 1000u
#define B5_PING_PERIOD_MS 70u
#define B5_ECHO_TIMEOUT_US 30000u
#define B5_STOP_MM 250u
#define B5_SIDE_CLEAR_MM 400u
#define B5_PROFILE_MM 800u
#define B5_FEEDBACK_MAX_MS 100u
#define B5_STOP_ACK_MS 500u
#define B5_SCAN_MAX_MS 45000u
#define B5_RECOVERY_MAX_MS 5000u
#define B5_RECOVERY_MAX_MM 500u
#define B5_INVALID_LIMIT 3u
/* Enable only after physical width validation passes. */
#define B5_WIDTH_VALIDATED 0
#endif
