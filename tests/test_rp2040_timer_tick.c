#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "timer_tick_policy.h"

int main(void)
{
    uint32_t configured = rp2040_timer_tick_value(12000000U);

    assert(configured == (RP2040_TIMER_TICK_ENABLE | 12U));
    assert((configured & RP2040_TIMER_TICK_CYCLES_MASK) == 12U);

    /* Reset CYCLES=0 is the hardware encoding for 512 cycles. */
    assert(12000000U / 512U == 23437U);
    assert(512U / 12U == 42U);

    assert(rp2040_timer_tick_value(999999U) == 0U);
    assert(rp2040_timer_tick_value(125000001U) == 0U);
    assert(rp2040_timer_tick_value(512000000U) == 0U);

    puts("PASS: RP2040 TIMER watchdog-tick divider policy");
    return 0;
}
