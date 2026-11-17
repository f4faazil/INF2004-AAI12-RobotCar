/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * freeroam_task.h
 * The open-space obstacle demo (ROBOT_FREEROAM = 1 in robot_config.h).  It replaces
 * the robot application: no line, no barcodes.  The car drives straight, stops when
 * the ultrasonic sensor sees an obstacle ahead, scans with the servo, goes round the
 * obstacle on the side the scan shows as open, comes back to its original path and
 * carries on.  START (GP20) begins, STOP (GP21) aborts at any moment.
 */
#ifndef FREEROAM_TASK_H
#define FREEROAM_TASK_H

#include <tk/tkernel.h>

/* Initialise the hardware and start the free-roam task. */
ER freeroam_start_task(void);

#endif /* FREEROAM_TASK_H */
