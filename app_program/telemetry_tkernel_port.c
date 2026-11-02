/*
 * This is the ONLY telemetry translation unit including tk/tkernel.h.
 * Deliberately DO NOT include telemetry_rtos.h or telemetry_inbox.h here.
 * The supplied micro T-Kernel aliases size_t to SZ (signed long), which
 * conflicts with the ARM C library's stddef.h (unsigned int).
 */
#include <tk/tkernel.h>
#include "telemetry_tkernel_port.h"

int telemetry_tkernel_mutex_create(void)
{
    static T_CMTX config = {
        .mtxatr = TA_TFIFO | TA_INHERIT
    };
    const ID id = tk_cre_mtx(&config);
    return (id > E_OK) ? (int)id : 0;
}

int telemetry_tkernel_mutex_try_lock(int mutex_id)
{
    return tk_loc_mtx((ID)mutex_id, TMO_POL) == E_OK;
}

void telemetry_tkernel_mutex_unlock(int mutex_id)
{
    (void)tk_unl_mtx((ID)mutex_id);
}

void telemetry_tkernel_mutex_delete(int mutex_id)
{
    (void)tk_del_mtx((ID)mutex_id);
}
