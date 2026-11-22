/*
 *  LSM303D-compatible GY-511 accelerometer registers.
 */

#ifndef LSM303_REGS_H
#define LSM303_REGS_H

#define LSM303_ACC_ADDR         (0x1DU)

#define LSM303_WHO_AM_I         (0x0FU)
#define LSM303_WHO_AM_I_VALUE   (0x49U)
#define LSM303_CTRL1            (0x20U)
#define LSM303_CTRL2            (0x21U)
#define LSM303_OUT_X_L          (0x28U)
#define LSM303_AUTO_INC         (0x80U)

#define LSM303_CTRL1_50HZ_XYZ_BDU (0x5FU)
#define LSM303_CTRL2_2G           (0x00U)

/* At +/-2 g, each raw count is 0.000061 g (0.061 mg). */
#define LSM303_MG_NUMERATOR     (61)
#define LSM303_MG_DENOMINATOR   (1000)

#endif /* LSM303_REGS_H */
