#ifndef RP2040_TIMER_TICK_POLICY_H
#define RP2040_TIMER_TICK_POLICY_H

#include <stdint.h>

#define RP2040_TIMER_TICK_HZ       1000000U
#define RP2040_TIMER_TICK_ENABLE   (1U << 9)
#define RP2040_TIMER_TICK_CYCLES_MASK 0x1ffU

/* RP2040 TIMER counts watchdog-tick events.  A cycle field of zero means
 * 512 clk_ref cycles, not "divide by one", so leaving the reset value intact
 * makes TIMER 42.666... times slow with the standard 12 MHz crystal. */
static inline uint32_t rp2040_timer_tick_value(uint32_t ref_hz)
{
    uint32_t cycles;

    if (ref_hz < RP2040_TIMER_TICK_HZ ||
        ref_hz % RP2040_TIMER_TICK_HZ != 0U) {
        return 0U;
    }
    cycles = ref_hz / RP2040_TIMER_TICK_HZ;
    if (cycles == 0U || cycles > RP2040_TIMER_TICK_CYCLES_MASK) {
        return 0U;
    }
    return RP2040_TIMER_TICK_ENABLE | cycles;
}

#endif
