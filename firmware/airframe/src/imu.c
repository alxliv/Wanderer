#include "imu.h"

#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "config.h"

#define LSM6DSO_WHO_AM_I       0x0F
#define LSM6DSO_WHO_AM_I_VALUE 0x6C
#define LSM6DSO_CTRL2_G        0x11
#define LSM6DSO_CTRL3_C        0x12
#define LSM6DSO_OUTZ_L_G       0x26

#define IMU_I2C_TIMEOUT_US     1000

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
    i2c_init(i2c1, IMU_I2C_BAUD);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);

    uint8_t who_am_i = 0;
    if (!read_registers(LSM6DSO_WHO_AM_I, &who_am_i, 1))
        return -1;
    if (who_am_i != LSM6DSO_WHO_AM_I_VALUE)
        return -2;

    // Reset first, then use coherent multi-byte reads and enable the gyro at
    // 104 Hz / +/-500 dps. The remaining filter and accel setup belongs to M1.
    if (!write_register(LSM6DSO_CTRL3_C, 0x01))
        return -3;
    sleep_ms(10);
    if (!write_register(LSM6DSO_CTRL3_C, 0x44))
        return -4;
    if (!write_register(LSM6DSO_CTRL2_G, 0x44))
        return -5;

    return 0;
}

int16_t imu_raw_gyro_z(void)
{
    uint8_t data[2];
    if (!read_registers(LSM6DSO_OUTZ_L_G, data, sizeof data))
        return 0;
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}
