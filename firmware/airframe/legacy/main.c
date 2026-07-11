/*
 * Wanderer - reflexive layer firmware (Pico 2 / RP2350).
 *
 * Phase 2 firmware: I2C peripheral, command watchdog, MDD10A open-loop motor
 * control, and PIO quadrature encoder telemetry. PID and ToF are later steps.
 */
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "i2c_registers.h"
#include "i2c_peripheral.h"
#include "config.h"
#include "encoder_math.h"
#include "encoders.h"
#include "motors.h"

#define CONTROL_PERIOD_US (1000000 / CONTROL_HZ)

/* Populate INFO + default CONTROL/CONFIG registers at boot. */
static void load_defaults(void) {
    /* INFO (read-only to the master, written once here) */
    i2cp_set_u8(REG_PROTOCOL_VERSION, WANDERER_PROTOCOL_VERSION);
    i2cp_set_u8(REG_FW_VERSION_MAJOR, FW_VERSION_MAJOR);
    i2cp_set_u8(REG_FW_VERSION_MINOR, FW_VERSION_MINOR);
    i2cp_set_u8(REG_DEVICE_ID,        WANDERER_DEVICE_ID);

    /* CONTROL / COMMAND defaults: safe idle, motors disabled */
    i2cp_set_u8(REG_CONTROL_MODE,     MODE_IDLE);
    i2cp_set_u8(REG_CONTROL_FLAGS,    0);
    i2cp_set_i16(REG_CMD_LEFT,        0);
    i2cp_set_i16(REG_CMD_RIGHT,       0);
    i2cp_set_u8(REG_WATCHDOG_TIMEOUT, DEFAULT_WATCHDOG_10MS);

    /* CONFIG defaults (online-tunable via I2C) */
    i2cp_set_f32(REG_TICKS_PER_METER, DEFAULT_TICKS_PER_METER);
    i2cp_set_f32(REG_PID_KP,          DEFAULT_PID_KP);
    i2cp_set_f32(REG_PID_KI,          DEFAULT_PID_KI);
    i2cp_set_f32(REG_PID_KD,          DEFAULT_PID_KD);
    i2cp_set_u16(REG_MAX_PWM,         DEFAULT_MAX_PWM);
    i2cp_set_u16(REG_OBSTACLE_STOP_MM, DEFAULT_OBSTACLE_STOP_MM);
}

/* Handle self-clearing action flags written by the master. */
static bool handle_control_flags(bool *odometry_reset) {
    uint8_t flags = i2cp_get_u8(REG_CONTROL_FLAGS);
    bool clear_faults = false;
    bool changed = false;

    *odometry_reset = false;

    if (flags & FLAG_CLEAR_FAULTS) {
        i2cp_set_u8(REG_FAULT, 0);
        clear_faults = true;
        flags &= ~FLAG_CLEAR_FAULTS;
        changed = true;
    }
    if (flags & FLAG_RESET_ODOM) {
        encoders_reset();
        i2cp_set_i32(REG_ENC_LEFT, 0);
        i2cp_set_i32(REG_ENC_RIGHT, 0);
        i2cp_set_i16(REG_MEAS_LEFT, 0);
        i2cp_set_i16(REG_MEAS_RIGHT, 0);
        *odometry_reset = true;
        flags &= ~FLAG_RESET_ODOM;
        changed = true;
    }
    if (changed) {
        i2cp_set_u8(REG_CONTROL_FLAGS, flags);
    }

    return clear_faults;
}

int main(void) {
    stdio_init_all();

#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

    i2cp_init(PI_I2C, PI_SDA_PIN, PI_SCL_PIN, PI_I2C_BAUD, WANDERER_I2C_ADDR);
    load_defaults();

    printf("\nWanderer reflexive layer fw %d.%d  proto %d  i2c addr 0x%02X\n",
           FW_VERSION_MAJOR, FW_VERSION_MINOR,
           WANDERER_PROTOCOL_VERSION, WANDERER_I2C_ADDR);

    motors_init();
    encoders_init();
    /* TODO(tof):      tof_init();      */

    absolute_time_t next_tick = make_timeout_time_us(CONTROL_PERIOD_US);
    absolute_time_t last_encoder_sample = get_absolute_time();
    uint32_t last_cmd_ms = to_ms_since_boot(get_absolute_time());
    bool watchdog_tripped = false;

    uint32_t loop_count = 0;
    absolute_time_t hz_mark = get_absolute_time();

    while (true) {
        sleep_until(next_tick);
        next_tick = delayed_by_us(next_tick, CONTROL_PERIOD_US);
        absolute_time_t now = get_absolute_time();
        uint32_t now_ms = to_ms_since_boot(now);

        /* 1. New command from the master refreshes the watchdog. */
        if (i2cp_take_command_written()) {
            last_cmd_ms = now_ms;
        }

        /* 2. Re-load tunables if the master touched the CONFIG region. */
        if (i2cp_take_config_written()) {
            /* TODO(pid): reload PID gains / ticks-per-meter / limits into the
             * controller once those modules exist. */
        }

        /* 3. Self-clearing action flags. */
        bool odometry_reset;
        bool clear_faults = handle_control_flags(&odometry_reset);

        /* Watchdog recovery is latched. CLEAR_FAULTS can clear the latch, but
         * motor output remains disabled until the host explicitly re-enables it
         * in a later command. */
        if (watchdog_tripped) {
            uint8_t flags = i2cp_get_u8(REG_CONTROL_FLAGS);
            if (flags & FLAG_MOTOR_ENABLE) {
                i2cp_set_u8(REG_CONTROL_FLAGS, flags & (uint8_t)~FLAG_MOTOR_ENABLE);
            }
            if (clear_faults) {
                watchdog_tripped = false;
            } else {
                i2cp_set_u8(REG_FAULT, i2cp_get_u8(REG_FAULT) | FT_WATCHDOG);
            }
        }

        /* 4. Command watchdog: stop if the tactical host goes quiet. */
        uint8_t wd_10ms = i2cp_get_u8(REG_WATCHDOG_TIMEOUT);
        if (wd_10ms != 0) {
            uint32_t timeout_ms = (uint32_t)wd_10ms * 10u;
            if ((now_ms - last_cmd_ms) > timeout_ms) {
                if (!watchdog_tripped) {
                    watchdog_tripped = true;
                    i2cp_set_u8(REG_CONTROL_MODE, MODE_IDLE);
                    i2cp_set_u8(REG_CONTROL_FLAGS,
                                i2cp_get_u8(REG_CONTROL_FLAGS) & (uint8_t)~FLAG_MOTOR_ENABLE);
                    i2cp_set_u8(REG_FAULT, i2cp_get_u8(REG_FAULT) | FT_WATCHDOG);
                    motors_stop();
                }
            }
        } else {
            watchdog_tripped = false; /* watchdog disabled */
        }

        /* 5. Read effective command state. */
        uint8_t mode  = i2cp_get_u8(REG_CONTROL_MODE);
        uint8_t flags = i2cp_get_u8(REG_CONTROL_FLAGS);
        bool enabled  = (flags & FLAG_MOTOR_ENABLE) && !watchdog_tripped;
        int16_t cmd_l = i2cp_get_i16(REG_CMD_LEFT);
        int16_t cmd_r = i2cp_get_i16(REG_CMD_RIGHT);
        uint16_t max_pwm = i2cp_get_u16(REG_MAX_PWM);

        /* 6. Apply open-loop drive. Velocity mode remains stopped until the
         * encoder/PID step is implemented. */
        if (enabled && mode == MODE_DIRECT_PWM) {
            motors_set(cmd_l, cmd_r, max_pwm);
        } else {
            motors_stop();
        }

        /* 7. Sample PIO-maintained encoder counts and derive wheel velocity. */
        encoder_sample_t encoder = encoders_sample();
        i2cp_set_i32(REG_ENC_LEFT, encoder.left_ticks);
        i2cp_set_i32(REG_ENC_RIGHT, encoder.right_ticks);

        if (odometry_reset) {
            i2cp_set_i16(REG_MEAS_LEFT, 0);
            i2cp_set_i16(REG_MEAS_RIGHT, 0);
        } else {
            int64_t elapsed = absolute_time_diff_us(last_encoder_sample, now);
            uint32_t elapsed_us =
                elapsed <= 0 ? 0u :
                elapsed > (int64_t)UINT32_MAX ? UINT32_MAX :
                (uint32_t)elapsed;
            float ticks_per_meter = i2cp_get_f32(REG_TICKS_PER_METER);

            i2cp_set_i16(
                REG_MEAS_LEFT,
                encoder_velocity_mm_s(encoder.left_delta, ticks_per_meter,
                                      elapsed_us));
            i2cp_set_i16(
                REG_MEAS_RIGHT,
                encoder_velocity_mm_s(encoder.right_delta, ticks_per_meter,
                                      elapsed_us));
        }
        last_encoder_sample = now;

        /* TODO(Phase 2 steps):
         *    - run per-wheel PID in VELOCITY mode
         *    - read ToF -> REG_TOF_FRONT_MM; obstacle reflex vs OBSTACLE_STOP_MM
         */

        /* 8. Publish STATUS telemetry. */
        uint8_t status = 0;
        if (enabled)              status |= ST_MOTORS_ENABLED;
        if (watchdog_tripped)     status |= ST_WATCHDOG_TRIPPED;
        if (i2cp_get_u8(REG_FAULT)) status |= ST_ANY_FAULT;
        status |= (uint8_t)((mode << ST_MODE_SHIFT) & ST_MODE_MASK);
        i2cp_set_u8(REG_STATUS, status);

        /* 9. Loop-rate health + heartbeat LED (once per second). */
        loop_count++;
        if (absolute_time_diff_us(hz_mark, get_absolute_time()) >= 1000000) {
            i2cp_set_u16(REG_LOOP_HZ, (uint16_t)loop_count);
            loop_count = 0;
            hz_mark = get_absolute_time();
#ifdef PICO_DEFAULT_LED_PIN
            gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
#endif
        }
    }
}
