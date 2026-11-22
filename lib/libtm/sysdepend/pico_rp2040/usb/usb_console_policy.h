#ifndef USB_CONSOLE_POLICY_H
#define USB_CONSOLE_POLICY_H

/* Bound one high-priority USB dispatch so a continuous controller event
 * stream cannot starve the lower-priority CYW43/lwIP owner on the same core. */
#define USB_EVENT_BURST_MAX 32U

static inline int usb_event_burst_has_room(unsigned int processed)
{
	return processed < USB_EVENT_BURST_MAX;
}

#endif
