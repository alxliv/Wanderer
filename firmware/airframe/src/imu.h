#ifndef WANDERER_IMU_H
#define WANDERER_IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bring up I2C1 and verify the MinIMU-9 v6's LSM6DSO gyro. Returns zero on
 * success; a negative value means the bus did not respond, WHO_AM_I was not
 * 0x6C, or the gyro could not be configured.
 */
int imu_init(void);

/* Read the LSM6DSO's uncorrected gyro-Z count for M0 diagnostics. */
int16_t imu_raw_gyro_z(void);

#ifdef __cplusplus
}
#endif

#endif /* WANDERER_IMU_H */
