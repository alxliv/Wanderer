#include "imu.h"

#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "config.h"

#define LSM6DSO_WHO_AM_I       0x0F
#define LSM6DSO_WHO_AM_I_VALUE 0x6C
#define LSM6DSO_CTRL1_XL       0x10
#define LSM6DSO_CTRL2_G        0x11
#define LSM6DSO_CTRL3_C        0x12
#define LSM6DSO_CTRL4_C        0x13
#define LSM6DSO_CTRL6_C        0x15
#define LSM6DSO_STATUS_REG     0x1E
#define LSM6DSO_OUTZ_L_G       0x26

#define IMU_I2C_TIMEOUT_US     1000
#define LSM6DSO_STATUS_GDA     0x02

/* +/-500 dps = 17.5 mdps/LSB; +/-2 g = 0.061 mg/LSB. */
#define GYRO_RAD_S_PER_LSB      0.00030543262f
#define ACCEL_M_S2_PER_LSB      0.00059825365f

static bool     s_initialized;
static int16_t  s_rawGyroZ;
static uint64_t s_lastFreshUs;
static float    s_yawBias;

static bool write_register(uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_write_timeout_us(i2c1, IMU_LSM6DSO_ADDR, data, sizeof data,
                                false, IMU_I2C_TIMEOUT_US) == (int)sizeof data;
}

static bool read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    if (i2c_write_timeout_us(i2c1, IMU_LSM6DSO_ADDR, &reg, 1, true,
                             IMU_I2C_TIMEOUT_US) != 1)
        return false;
    return i2c_read_timeout_us(i2c1, IMU_LSM6DSO_ADDR, data, length, false,
                               IMU_I2C_TIMEOUT_US) == (int)length;
}

int imu_init(void)
{
    s_initialized = false;
    s_rawGyroZ = 0;
    s_lastFreshUs = 0;
    s_yawBias = 0.0f;

    i2c_init(i2c1, IMU_I2C_BAUD);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);

    uint8_t who_am_i = 0;
    if (!read_registers(LSM6DSO_WHO_AM_I, &who_am_i, 1))
        return -1;
    if (who_am_i != LSM6DSO_WHO_AM_I_VALUE)
        return -2;

    // Reset first, then use coherent multi-byte reads and enable gyro and
    // accelerometer at 104 Hz. CTRL6_C FTYPE=101 gives a 31 Hz gyro LPF1
    // cutoff at this ODR, below the 50 Hz control-loop Nyquist frequency.
    if (!write_register(LSM6DSO_CTRL3_C, 0x01))
        return -3;
    sleep_ms(10);
    if (!write_register(LSM6DSO_CTRL3_C, 0x44))
        return -4;
    if (!write_register(LSM6DSO_CTRL2_G, 0x44))
        return -5;
    if (!write_register(LSM6DSO_CTRL1_XL, 0x40))
        return -6;
    if (!write_register(LSM6DSO_CTRL4_C, 0x02))
        return -7;
    if (!write_register(LSM6DSO_CTRL6_C, 0x05))
        return -8;

    s_initialized = true;
    return 0;
}

imu_sample_t imu_sample(void)
{
    imu_sample_t sample = {
        .yaw_rate = 0.0f,
        .az = 0.0f,
        .t_us = to_us_since_boot(get_absolute_time()),
        .fresh = false,
    };
    if (!s_initialized)
        return sample;

    uint8_t status;
    if (!read_registers(LSM6DSO_STATUS_REG, &status, 1))
        return sample;
    if ((status & LSM6DSO_STATUS_GDA) == 0)
        return sample;

    // OUTZ_L_G through OUTZ_H_A is eight contiguous bytes. BDU was enabled
    // at init, so neither signed pair can straddle a sensor update.
    uint8_t data[8];
    if (!read_registers(LSM6DSO_OUTZ_L_G, data, sizeof data))
        return sample;

    const int16_t gyro_z = (int16_t)((uint16_t)data[0] |
                                     ((uint16_t)data[1] << 8));
    const int16_t accel_z = (int16_t)((uint16_t)data[6] |
                                      ((uint16_t)data[7] << 8));
    s_rawGyroZ = gyro_z;
    sample.t_us = to_us_since_boot(get_absolute_time());
    s_lastFreshUs = sample.t_us;
    sample.yaw_rate = (float)gyro_z * IMU_YAW_SIGN * IMU_SCALE *
                      GYRO_RAD_S_PER_LSB - s_yawBias;
    sample.az = (float)accel_z * ACCEL_M_S2_PER_LSB;
    sample.fresh = true;
    return sample;
}

bool imu_healthy(void)
{
    if (!s_initialized || s_lastFreshUs == 0)
        return false;
    const uint64_t now_us = to_us_since_boot(get_absolute_time());
    return now_us - s_lastFreshUs <= (uint64_t)IMU_STALE_MS * 1000u;
}

int16_t imu_raw_gyro_z(void) { return s_rawGyroZ; }

void imu_set_yaw_bias(float bias_rad_s) { s_yawBias = bias_rad_s; }
float imu_yaw_bias(void) { return s_yawBias; }

bool imu_calibrate_bias(float *mean_rad_s, float *stddev_rad_s)
{
    enum { SAMPLE_COUNT = 512 };
    if (!s_initialized)
        return false;
    float mean = 0.0f;
    float sum_squares = 0.0f;
    for (unsigned i = 0; i < SAMPLE_COUNT; ++i) {
        imu_sample_t sample;
        const uint64_t deadline = to_us_since_boot(get_absolute_time()) + 50000u;
        do {
            sample = imu_sample();
            if (!sample.fresh)
                sleep_us(500);
        } while (!sample.fresh
                 && to_us_since_boot(get_absolute_time()) < deadline);
        if (!sample.fresh)
            return false;

        const float raw_rate = sample.yaw_rate + s_yawBias;
        const float delta = raw_rate - mean;
        mean += delta / (float)(i + 1);
        sum_squares += delta * (raw_rate - mean);
    }
    if (mean_rad_s)
        *mean_rad_s = mean;
    if (stddev_rad_s)
        *stddev_rad_s = sqrtf(sum_squares / (float)(SAMPLE_COUNT - 1));
    return true;
}
