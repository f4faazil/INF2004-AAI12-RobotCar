/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * imu_test_task.h
 * The IMU bring-up application (ROBOT_IMU_TEST = 1 in robot_config.h).  It
 * replaces the robot application: one task, driven by the START button, that
 * checks the accelerometer, finds the forward axis and its sign, and runs the
 * hump detector over a real 1 m drive.
 */
#ifndef IMU_TEST_TASK_H
#define IMU_TEST_TASK_H

#include <tk/tkernel.h>

/* Initialise the hardware and start the IMU test task. */
ER imu_test_start_task(void);

#endif /* IMU_TEST_TASK_H */
