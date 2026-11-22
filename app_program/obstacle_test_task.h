/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * obstacle_test_task.h
 * The ultrasonic and servo bring-up application (ROBOT_OBSTACLE_TEST = 1 in
 * robot_config.h).  It replaces the robot application: one task, driven by the
 * START button, that measures the HC-SR04 at known distances, checks the
 * no-echo handling, sweeps the servo to its limits and runs the real scan,
 * profile and bypass planner.  The motors are never driven.
 */
#ifndef OBSTACLE_TEST_TASK_H
#define OBSTACLE_TEST_TASK_H

#include <tk/tkernel.h>

/* Initialise the hardware and start the obstacle test task. */
ER obstacle_test_start_task(void);

#endif /* OBSTACLE_TEST_TASK_H */
