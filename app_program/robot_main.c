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
#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

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
#if ROBOT_START_DELAY_MS
    ER err;

    tm_printf((UB *)"[boot] waiting %d ms before starting the robot tasks (debug)\n", (INT)ROBOT_START_DELAY_MS);
    {
        UW waited;

        for (waited = 0U; waited < ROBOT_START_DELAY_MS; waited += 1000U) {
#if TM_WIFI_CYW43
            static T_CYW43_UTK_STATUS w;                /* static: the init task's stack is small */

            cyw43_utk_get_status(&w);
            tm_printf((UB *)"[boot] t=%us radio=%s networks-seen=%u join-state=%u attempts=%u failed=%u last-rc=%d link=%u dhcp=%u mqtt=%u\n",
                      (UINT)(waited / 1000U), (UB *)(w.ready ? "ready" : "NOT-READY"), (UINT)w.scan_count, (UINT)w.join_state,
                      (UINT)w.join_attempts, (UINT)w.join_failures, (INT)w.join_result, (UINT)w.link_up, (UINT)w.dhcp_complete,
                      (UINT)w.mqtt_state);
#endif
            tk_dly_tsk(1000U);
        }
    }
    tm_printf((UB *)"[boot] starting the robot tasks now\n");
    err = robot_start();
#else
    ER err = robot_start();
#endif
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
