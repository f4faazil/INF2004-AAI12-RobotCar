/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * motor_cal_task.h
 * The motor calibration application (ROBOT_MOTOR_CAL = 1 in robot_config.h).
 * It replaces the robot application: only one task runs, it is the only
 * writer of the motor PWM, and it is driven by the START button.
 */
#ifndef MOTOR_CAL_TASK_H
#define MOTOR_CAL_TASK_H

#include <tk/tkernel.h>

/* Initialise the hardware and start the calibration task. */
ER motor_cal_start_task(void);

#endif /* MOTOR_CAL_TASK_H */
