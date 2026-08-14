#ifndef WANDERER_IMU_H
#define WANDERER_IMU_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bring up I2C1 and verify the MinIMU-9 v6's LSM6DSO gyro. Returns zero on
 * success; a negative value means the bus did not respond, WHO_AM_I was not
 * 0x6C, or the gyro could not be configured.
 */
typedef struct {
    float    yaw_rate;  /* rad/s, right-turn-positive, bias-corrected */
    float    az;        /* m/s^2, for M2's stillness detector */
    uint64_t t_us;      /* RP2350 timestamp of this sample */
    bool     fresh;     /* false if no new gyro sample was available */
} imu_sample_t;

int imu_init(void);

/* Poll the LSM6DSO once per control tick. */
imu_sample_t imu_sample(void);

/* False when no fresh sample has arrived within IMU_STALE_MS. */
bool imu_healthy(void);

/* The estimator owns the bias policy; the driver applies its current value. */
void  imu_set_yaw_bias(float bias_rad_s);
float imu_yaw_bias(void);

/* Average 512 fresh raw gyro-Z samples. Does not change the current bias. */
bool imu_calibrate_bias(float *mean_rad_s, float *stddev_rad_s);

/* The latest LSM6DSO gyro-Z count, uncorrected, for diagnostics. */
int16_t imu_raw_gyro_z(void);

#ifdef __cplusplus
}
#endif

#endif /* WANDERER_IMU_H */
