/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * robot_tasks.h
 * Entry point of the robot application: creates the kernel objects and tasks
 * and starts the 1 kHz tick.
 */
#ifndef ROBOT_TASKS_H
#define ROBOT_TASKS_H

#include <tk/tkernel.h>

/* Called once from usermain().  Returns E_OK or the first kernel error. */
ER robot_start(void);

#endif /* ROBOT_TASKS_H */
