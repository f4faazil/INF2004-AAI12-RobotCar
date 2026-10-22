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

#include "robot_tasks.h"

EXPORT INT usermain(void)
{
    ER err = robot_start();

    if (err < E_OK) {
        tm_printf((UB *)"[robot] start-up failed, er=%d\n", err);
    }
    /* The initial task has nothing left to do.  It must not return: on
     * return the kernel shuts the system down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}

#endif  /* ROBOT_FIRMWARE */
