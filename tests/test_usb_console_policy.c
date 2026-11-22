#include <assert.h>
#include <stdio.h>

#include "usb_console_policy.h"

int main(void)
{
	assert(USB_EVENT_BURST_MAX > 0U);
	assert(usb_event_burst_has_room(0U));
	assert(usb_event_burst_has_room(USB_EVENT_BURST_MAX - 1U));
	assert(!usb_event_burst_has_room(USB_EVENT_BURST_MAX));
	assert(!usb_event_burst_has_room(USB_EVENT_BURST_MAX + 1U));
	puts("PASS: USB event burst fairness policy");
	return 0;
}
