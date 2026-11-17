/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * robot_main.c
 * usermain() of the robot application.  When ROBOT_FIRMWARE is 0 the
 * lecturer's demo (app_main.c) provides usermain() instead.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "robot_config.h"

#if ROBOT_FIRMWARE

#include "freeroam_task.h"
#include "imu_test_task.h"
#include "obstacle_test_task.h"
#include "motor_cal_task.h"
#include "robot_tasks.h"

EXPORT INT usermain(void)
{
#if ROBOT_FREEROAM
    ER err = freeroam_start_task();
#elif ROBOT_OBSTACLE_TEST
    ER err = obstacle_test_start_task();
#elif ROBOT_IMU_TEST
    ER err = imu_test_start_task();
#elif ROBOT_MOTOR_CAL
    ER err = motor_cal_start_task();
#else
    ER err = robot_start();
#endif

    if (err < E_OK) {
        tm_printf((UB *)"[robot] start-up failed, er=%d\n", err);
    }
    /* The initial task has nothing left to do.  It must not return: on
     * return the kernel shuts the system down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}

#endif  /* ROBOT_FIRMWARE */
