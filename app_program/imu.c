/*
 *  LSM303D-compatible GY-511 sensor bring-up and acceleration test.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../device/include/device.h"

#include "usb_console_compat.h"
#include "imu.h"
#include "lsm303_regs.h"

#define IMU_DEV_NAME        "iicb"   /* I2C1: GP26 SDA, GP27 SCL */
#define IMU_PERIOD_MS       (20)     /* Sensor output rate: 50 Hz */
#define IMU_PRINT_EVERY     (10)     /* Print every 200 ms */
#define IMU_MAX_FAILS       (3)
#define IMU_BASELINE_SAMPLES (50)
#define IMU_EVENT_ON_MG      (50)    /* Provisional; tune from controlled runs. */
#define IMU_EVENT_OFF_MG     (25)    /* Hysteresis below the onset threshold. */
#define IMU_EVENT_MIN_SAMPLES (3)
#define IMU_EVENT_QUIET_SAMPLES (5)

LOCAL ID    imu_dd = -1;
EXPORT volatile UW imu_debug_stage;
EXPORT volatile INT imu_debug_error;
EXPORT volatile UW imu_debug_init_step;
EXPORT volatile UW imu_debug_whoami;
EXPORT volatile imu_event_t imu_latest_event;
EXPORT volatile UW imu_largest_peak_deviation_mg;

LOCAL ER imu_write_reg(UB reg, UB val)
{
    UB  buf[2];
    SZ  asz;

    buf[0] = reg;
    buf[1] = val;
    return tk_swri_dev(imu_dd, LSM303_ACC_ADDR, buf, sizeof(buf), &asz);
}

LOCAL ER imu_read_regs(UB reg, UB *data, UW size)
{
    T_I2C_EXEC  ex;
    UB          command = reg;
    SZ          asz;

    ex.sadr     = LSM303_ACC_ADDR;
    ex.snd_size = 1;
    ex.snd_data = &command;
    ex.rcv_size = size;
    ex.rcv_data = data;

    return tk_swri_dev(imu_dd, TDN_I2C_EXEC, &ex, sizeof(ex), &asz);
}

LOCAL ER imu_init(void)
{
    UB  whoami = 0;
    ER  er;

    imu_debug_init_step = 1;
    imu_dd = tk_opn_dev((UB *)IMU_DEV_NAME, TD_UPDATE);
    if(imu_dd < E_OK) {
        imu_debug_error = imu_dd;
        return imu_dd;
    }

    imu_debug_init_step = 2;
    er = imu_read_regs(LSM303_WHO_AM_I, &whoami, 1);
    if(er < E_OK) {
        imu_debug_error = er;
        return er;
    }
    imu_debug_whoami = whoami;
    if(whoami != LSM303_WHO_AM_I_VALUE) {
        imu_debug_error = E_IO;
        tm_printf((UB *)"[imu] unexpected WHO_AM_I=0x%x (expected 0x%x)\n",
                  whoami, LSM303_WHO_AM_I_VALUE);
        return E_IO;
    }
    tm_printf((UB *)"[imu] WHO_AM_I=0x%x\n", whoami);

    imu_debug_init_step = 3;
    er = imu_write_reg(LSM303_CTRL2, LSM303_CTRL2_2G);
    if(er < E_OK) {
        imu_debug_error = er;
        return er;
    }

    imu_debug_init_step = 4;
    er = imu_write_reg(LSM303_CTRL1, LSM303_CTRL1_50HZ_XYZ_BDU);
    if(er < E_OK) {
        imu_debug_error = er;
        return er;
    }

    tk_dly_tsk(100);
    imu_debug_init_step = 5;
    imu_debug_error = E_OK;
    return E_OK;
}

LOCAL INT imu_raw_to_mg(H raw)
{
    INT scaled = (INT)raw * LSM303_MG_NUMERATOR;

    if(scaled >= 0) {
        scaled += LSM303_MG_DENOMINATOR / 2;
    } else {
        scaled -= LSM303_MG_DENOMINATOR / 2;
    }
    return scaled / LSM303_MG_DENOMINATOR;
}

LOCAL void imu_print_axis(const char *name, INT mg)
{
    INT magnitude = (mg < 0) ? -mg : mg;

    tm_printf((UB *)"%s%s%d.%03d g",
              name, (mg < 0) ? "-" : "",
              magnitude / 1000, magnitude % 1000);
}

LOCAL ER imu_read_xyz(INT *x_mg, INT *y_mg, INT *z_mg)
{
    UB raw[6];
    H raw_x, raw_y, raw_z;
    ER er;

    er = imu_read_regs((UB)(LSM303_OUT_X_L | LSM303_AUTO_INC), raw, sizeof(raw));
    if(er < E_OK) {
        return er;
    }

    raw_x = (H)(((UW)raw[1] << 8) | raw[0]);
    raw_y = (H)(((UW)raw[3] << 8) | raw[2]);
    raw_z = (H)(((UW)raw[5] << 8) | raw[4]);
    *x_mg = imu_raw_to_mg(raw_x);
    *y_mg = imu_raw_to_mg(raw_y);
    *z_mg = imu_raw_to_mg(raw_z);
    return E_OK;
}

LOCAL UW imu_isqrt(UW value)
{
    UW result = 0;
    UW bit = (UW)1 << 30;

    while(bit > value) {
        bit >>= 2;
    }
    while(bit != 0) {
        if(value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

LOCAL UW imu_deviation_mg(INT x, INT y, INT z,
                          INT base_x, INT base_y, INT base_z)
{
    INT dx = x - base_x;
    INT dy = y - base_y;
    INT dz = z - base_z;
    UW square_sum = (UW)(dx * dx) + (UW)(dy * dy) + (UW)(dz * dz);

    return imu_isqrt(square_sum);
}

LOCAL void imu_finish_event(UW duration_samples, UW peak_mg)
{
    UW duration_ms = duration_samples * IMU_PERIOD_MS;

    if(duration_samples < IMU_EVENT_MIN_SAMPLES) {
        return;
    }
    if(peak_mg > imu_largest_peak_deviation_mg) {
        imu_largest_peak_deviation_mg = peak_mg;
    }

    imu_latest_event.duration_ms = duration_ms;
    imu_latest_event.peak_deviation_mg = peak_mg;
    imu_latest_event.packed = hump_pack(IMU_ST_REPORT, IMU_HEIGHT_OOR,
                                        IMU_UNAVAILABLE);
    imu_latest_event.sequence++;
    tm_printf((UB *)"[imu] HUMP candidate seq=%u duration=%u ms"
              " peak_deviation=%u mg height=unavailable (calibration required)\n",
              imu_latest_event.sequence, duration_ms, peak_mg);
    tm_printf((UB *)"[imu] largest peak deviation=%u mg\n",
              imu_largest_peak_deviation_mg);
}

EXPORT void imu_task(INT stacd, void *exinf)
{
    INT x, y, z;
    INT base_x = 0, base_y = 0, base_z = 0;
    INT sum_x = 0, sum_y = 0, sum_z = 0;
    UW  fails = 0, print_samples = 0, calibrated = 0;
    UW  event_samples = 0, quiet_samples = 0, peak_mg = 0;
    UW  calibration_samples = 0, deviation, elapsed_samples;
    BOOL event_active = FALSE;
    INT i;
    ER  er;

    (void)stacd;
    (void)exinf;

    imu_debug_stage = 1;
    imu_latest_event.sequence = 0;
    imu_latest_event.duration_ms = 0;
    imu_latest_event.peak_deviation_mg = 0;
    imu_latest_event.packed = hump_pack(IMU_ST_FLAT, IMU_HEIGHT_OOR,
                                        IMU_UNAVAILABLE);
    tm_printf((UB *)"[imu] task entered\n");

    for(i = 0; (i < 200) && (tm_usb_state() < 2U); i++) {
        tk_dly_tsk(100);
    }
    tk_dly_tsk(2000);

    imu_debug_stage = 2;
    er = imu_init();
    if(er < E_OK) {
        imu_debug_stage = 4;
        tm_printf((UB *)"[imu] init FAILED step=%u er=%d who=0x%x\n",
                  imu_debug_init_step, er, imu_debug_whoami);
        for(;;) {
            tk_dly_tsk(1000);
        }
    }

    imu_debug_stage = 3;
    tm_printf((UB *)"[imu] init OK on %s addr 0x%x; 50 Hz, +/-2 g\n",
              IMU_DEV_NAME, LSM303_ACC_ADDR);
    tm_printf((UB *)"[imu] keep car still for gravity baseline (%u samples)\n",
              IMU_BASELINE_SAMPLES);

    for(;;) {
        er = imu_read_xyz(&x, &y, &z);
        if(er < E_OK) {
            fails++;
            tm_printf((UB *)"[imu] read fail %u er=%d\n", fails, er);
            if(fails >= IMU_MAX_FAILS) {
                imu_debug_stage = 4;
                imu_debug_error = er;
                tm_printf((UB *)"[imu] DEGRADED: sensor read unavailable\n");
            }
        } else {
            fails = 0;
            if(!calibrated) {
                sum_x += x;
                sum_y += y;
                sum_z += z;
                if(++calibration_samples >= IMU_BASELINE_SAMPLES) {
                    base_x = sum_x / (INT)calibration_samples;
                    base_y = sum_y / (INT)calibration_samples;
                    base_z = sum_z / (INT)calibration_samples;
                    calibrated = 1;
                    tm_printf((UB *)"[imu] gravity baseline x=%d y=%d z=%d mg\n",
                              base_x, base_y, base_z);
                }
            } else {
                deviation = imu_deviation_mg(x, y, z,
                                             base_x, base_y, base_z);
                if(++print_samples >= IMU_PRINT_EVERY) {
                    print_samples = 0;
                    tm_printf((UB *)"[imu] x=");
                    imu_print_axis("", x);
                    tm_printf((UB *)" y=");
                    imu_print_axis("", y);
                    tm_printf((UB *)" z=");
                    imu_print_axis("", z);
                    tm_printf((UB *)" deviation=%u mg\n", deviation);
                }

                if(!event_active) {
                    if(deviation >= IMU_EVENT_ON_MG) {
                        event_active = TRUE;
                        event_samples = 1;
                        quiet_samples = 0;
                        peak_mg = deviation;
                    }
                } else {
                    elapsed_samples = event_samples + quiet_samples;
                    if(deviation > peak_mg) {
                        peak_mg = deviation;
                    }
                    if(deviation <= IMU_EVENT_OFF_MG) {
                        quiet_samples++;
                    } else {
                        event_samples = elapsed_samples + 1;
                        quiet_samples = 0;
                    }
                    if(quiet_samples >= IMU_EVENT_QUIET_SAMPLES) {
                        imu_finish_event(event_samples, peak_mg);
                        event_active = FALSE;
                        event_samples = 0;
                        quiet_samples = 0;
                        peak_mg = 0;
                    }
                }
            }
        }
        tk_dly_tsk(IMU_PERIOD_MS);
    }
}
