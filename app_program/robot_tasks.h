/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * robot_tasks.h
 * Entry point of the robot application: creates the kernel objects and tasks
 * and starts the 1 kHz tick.
 */
#ifndef ROBOT_TASKS_H
#define ROBOT_TASKS_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "imu.h"

/* Called once from usermain().  Returns E_OK or the first kernel error. */
ER robot_start(void);

/* Find and configure the accelerometer on I2C1 (LSM303D at 0x1D, else LSM303DLHC
 * at 0x19).  Also used by the IMU bring-up application. */
bool robot_imu_probe(imu_variant_t *variant, uint8_t *addr);

#endif /* ROBOT_TASKS_H */
