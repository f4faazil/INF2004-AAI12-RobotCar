#ifndef B5_UTK_H
#define B5_UTK_H
#include <tk/tkernel.h>
#include "b5.h"
/* Call once from usermain, before starting any consumer tasks. */
ER b5_start(void);
/* TASK context only. Nonblocking, mutex-protected complete packet copies.
 * false means busy/not initialised/invalid input. Retry later; never spin.
 * The Vehicle Controller is the sole feedback producer in mode 4.
 * at_ms must represent the age of the underlying motion/line snapshot,
 * not a refreshed timestamp on old measurements. */
bool b5_submit_feedback(unsigned mode, bool stopped, bool motion_valid,
                        bool line_valid, bool line_found, unsigned travel_mm,
                        uint32_t measured_at_ms);
bool b5_read_packet(uint8_t out[B5_STATUS_BYTES]);
bool b5_read_scan(uint8_t out[B5_SCAN_BYTES]);
uint32_t b5_now_ms(void);
#endif
