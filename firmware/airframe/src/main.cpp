// Wanderer airframe main -- the tactical layer behind the Pico2 UART.
//
// Wiring (all shared code, no logic of its own here):
//   UART0 bytes  -> cockpit_feed() -> cockpit_handler -> tac_* (FSM)
//   FSM change   -> `!fault`/`!state` lines -> UART0
//   tac targets  -> open-loop per-mille -> motors_set()
//   encoders     -> odometry provider -> `=ok get_odometry ...`
//
// Motor control runs in a simple wheel-speed closed loop: encoder-derived
// wheel velocity is compared against the tactical target each control tick,
// and the PWM command is trimmed to reduce the error. The cockpit protocol
// stays the same; only the low-level actuation path changes.
//
// stdio stays on USB CDC, which now carries the SYSTEM BACKDOOR (arch 3a)
// as well as bench logs (`*` lines); the Pico2 UART carries ONLY cockpit
// protocol lines. Two ports, two line assemblers, one FSM -- and one motion
// lease (tac_dev_*) deciding which of them may move a wheel.

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/uart.h"

#include "backdoor_handler.h"
#include "cockpit_handler.h"
#include "tactical.h"
#include "config.h"
#include "encoders.h"
#include "encoder_math.h"
#include "heading.h"
#include "imu.h"
#include "motors.h"

static uint64_t now_us(void)
{
    return to_us_since_boot(get_absolute_time());
}

static int16_t s_left_motor_command = 0;
static int16_t s_right_motor_command = 0;
static float s_left_motor_integral = 0.0f;
static float s_right_motor_integral = 0.0f;

// ---- cockpit transport -----------------------------------------------------

static void cockpit_line_out(const char *line)
{
    uart_puts(PICO2_UART, line);
    uart_puts(PICO2_UART, "\r\n");
}

static void pico2_uart_init(void)
{
    uart_init(PICO2_UART, PICO2_UART_BAUD);
    gpio_set_function(PICO2_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO2_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(PICO2_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(PICO2_UART, true);
}

// ---- backdoor transport (USB CDC) -----------------------------------------

// Bench logs and backdoor replies share the CDC port; both are line-oriented
// and the `*` sigil keeps logs classifiable as ignorable by any parser.
static void backdoor_line_out(const char *line)
{
    printf("%s\r\n", line);
}

// Raw per-mille held between control periods. Only ever applied while the
// backdoor holds the motion lease AND a wiggle is outstanding.
static int16_t s_bdLeft, s_bdRight;

static void backdoor_motor_out(int16_t left_permille, int16_t right_permille)
{
    s_bdLeft = left_permille;
    s_bdRight = right_permille;
}

static void backdoor_encoders(int32_t *left_ticks, int32_t *right_ticks);

// The IMU is polled only by the 100 Hz control loop. Backdoor reads use this
// cache, so bench traffic cannot create an extra I2C transaction or jitter it.
static imu_sample_t s_imu_sample;

static float backdoor_imu_rate_degrees_s(void)
{
    return s_imu_sample.yaw_rate * 180.0f / PI;
}

static bool backdoor_imu_is_healthy(void) { return imu_healthy(); }
static float backdoor_imu_bias_degrees_s(void)
{
    return heading_bias() * 180.0f / PI;
}
static float backdoor_imu_heading_degrees(void)
{
    return heading_get() * 180.0f / PI;
}
static bool backdoor_imu_calibrate(float *mean_deg_s, float *stddev_deg_s)
{
    if (tac_state() != TacticalState::Safe)
        return false;
    float mean, stddev;
    if (!heading_calibrate(&mean, &stddev))
        return false;
    *mean_deg_s = mean * 180.0f / PI;
    *stddev_deg_s = stddev * 180.0f / PI;
    return true;
}

static void cockpit_heading_provider(float *psi, float *rate, float *bias,
                                     bool *valid)
{
    *psi = heading_get();
    *rate = heading_rate();
    *bias = heading_bias();
    *valid = heading_valid();
}

static bool cockpit_imu_healthy_provider(void) { return imu_healthy(); }

// ---- odometry --------------------------------------------------------------

// Updated by the control loop, read by the cockpit's get_odometry.
static encoder_sample_t s_odom_sample;
static int16_t s_vl_mm_s, s_vr_mm_s;

static void odometry_provider(int32_t *lt, int32_t *rt, float *vl, float *vr)
{
    *lt = s_odom_sample.left_ticks;
    *rt = s_odom_sample.right_ticks;
    *vl = (float)s_vl_mm_s / 1000.0f;
    *vr = (float)s_vr_mm_s / 1000.0f;
}

// The backdoor reads the same cached sample, so `enc` and `get_odometry`
// can never disagree about where the wheels are.
static void backdoor_encoders(int32_t *left_ticks, int32_t *right_ticks)
{
    *left_ticks = s_odom_sample.left_ticks;
    *right_ticks = s_odom_sample.right_ticks;
}

// ---- open-loop motor mapping ----------------------------------------------

static int16_t permille_from_mm_s(int16_t mm_s)
{
    int32_t p = (int32_t)mm_s * 1000 / DEFAULT_MAX_SPEED_MM_S;
    if (p >  1000) p =  1000;
    if (p < -1000) p = -1000;
    return (int16_t)p;
}

// ---- main ------------------------------------------------------------------

int main(void)
{
    stdio_init_all();          // USB CDC: bench logs only

    pico2_uart_init();
    motors_init();
    encoders_init();
    encoders_reset();
    const int imu_rc = imu_init();
    heading_init();

    tac_init();
    cockpit_init(cockpit_line_out, FW_VERSION_MAJOR, FW_VERSION_MINOR,
                 DEFAULT_TICKS_PER_METER, TRACK_WIDTH_M / 2.0f,
                 DEFAULT_MAX_SPEED_MM_S / 1000.0f);
    cockpit_set_odometry_provider(odometry_provider);
    cockpit_set_heading_provider(cockpit_heading_provider, heading_zero);
    cockpit_set_imu_healthy_provider(cockpit_imu_healthy_provider);
    cockpit_set_turn_config(TURN_RATE_RAD_S, TURN_OVERSHOOT_RAD,
                            TURN_LINEAR_LIMIT_M_S);
    cockpit_set_move_config(HEADING_KP, HEADING_KI, HEADING_KD,
                            HEADING_I_MAX_RAD_S, HEADING_OMEGA_MAX_RAD_S);
    cockpit_set_motor_config(MOTOR_LEFT_GAIN_PERMILLE,
                             MOTOR_RIGHT_GAIN_PERMILLE,
                             MOTOR_DEADBAND_LEFT, MOTOR_DEADBAND_RIGHT);
    // Relay sink deliberately not set: `^` payloads are dropped until the
    // RF modem hat lands (cockpit spec section 4).

    backdoor_init(backdoor_line_out, FW_VERSION_MAJOR, FW_VERSION_MINOR);
    backdoor_set_motor_sink(backdoor_motor_out);
    backdoor_set_encoder_provider(backdoor_encoders, encoders_reset);
    if (imu_rc == 0)
        backdoor_set_imu_raw_provider(imu_raw_gyro_z);
    if (imu_rc == 0)
        backdoor_set_imu_status_providers(backdoor_imu_rate_degrees_s,
                                           backdoor_imu_is_healthy);
    if (imu_rc == 0)
        backdoor_set_imu_estimator_providers(backdoor_imu_bias_degrees_s,
                                              backdoor_imu_heading_degrees,
                                              backdoor_imu_calibrate);
    backdoor_set_config(ENC_LEFT_SIGN, ENC_RIGHT_SIGN,
                        MOTOR_LEFT_SIGN, MOTOR_RIGHT_SIGN,
                        DEFAULT_TICKS_PER_METER, DEFAULT_MAX_SPEED_MM_S,
                        IMU_YAW_SIGN, IMU_SCALE);

    printf("*airframe fw %u.%u cockpit on pico2 uart0 @%u\r\n",
           FW_VERSION_MAJOR, FW_VERSION_MINOR, (unsigned)PICO2_UART_BAUD);
    printf("*backdoor on usb cdc -- type `help`\r\n");
    if (imu_rc == 0)
        printf("*imu LSM6DSO ready on i2c1 bias=%.3f sigma=%.3f deg/s\r\n",
               (double)backdoor_imu_bias_degrees_s(),
               (double)(heading_calibration_stddev() * 180.0f / PI));
    else
        printf("*imu init failed rc=%d\r\n", imu_rc);

    bool usb_was_connected = stdio_usb_connected();

    const uint32_t control_period_us = 1000000u / CONTROL_HZ;
    uint64_t next_control = now_us();

    while (true) {
        // Pump every waiting cockpit byte; time-stamp at arrival.
        while (uart_is_readable(PICO2_UART))
            cockpit_feed((char)uart_getc(PICO2_UART), now_us());

        // Pump the backdoor. Non-blocking: getchar_timeout_us(0) returns
        // PICO_ERROR_TIMEOUT when the CDC buffer is empty, so the control
        // loop is never held up by an idle bench terminal.
        int ch;
        while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT)
            backdoor_feed((char)ch, now_us());

        // An unplugged cable must stop the wheels, not leave them latched at
        // whatever duty the last wiggle set.
        const bool usb_now = stdio_usb_connected();
        if (usb_was_connected && !usb_now)
            backdoor_abort();
        usb_was_connected = usb_now;

        const uint64_t t = now_us();
        if (t >= next_control) {
            next_control += control_period_us;

            // Odometry: sample and derive wheel velocities over the period.
            s_odom_sample = encoders_sample();
            s_vl_mm_s = encoder_velocity_mm_s(s_odom_sample.left_delta,
                                              DEFAULT_TICKS_PER_METER,
                                              control_period_us);
            s_vr_mm_s = encoder_velocity_mm_s(s_odom_sample.right_delta,
                                              DEFAULT_TICKS_PER_METER,
                                              control_period_us);

            s_imu_sample = imu_sample();
            heading_update(&s_imu_sample, &s_odom_sample,
                           cockpit_procedure_active());

            // FSM housekeeping: deadman and fallback ramp.
            tac_tick(t);
            cockpit_tick(t);

            // Backdoor housekeeping BEFORE the mux: expires the wiggle
            // deadline and zeroes s_bd* on lease loss, so the decision below
            // always reads a current command.
            backdoor_tick(t);

            // Motor mux. The two motion paths are mutually exclusive by
            // construction: the bench path requires the dev lease, and the
            // lease can only be held while the FSM is SAFE (where
            // tac_motors_enabled() is false). Bench first, so that even a
            // hypothetical overlap resolves toward the operator standing next
            // to the robot rather than the absent Pilot.
            //
            // SAFE/FAULT gate to zero via motors_stop so the driver's outputs
            // are unambiguous, not merely zero-valued.
            if (tac_dev_active() && backdoor_wiggle_active()) {
                s_left_motor_command = s_bdLeft;
                s_right_motor_command = s_bdRight;
                s_left_motor_integral = 0.0f;
                s_right_motor_integral = 0.0f;
                motors_set(s_left_motor_command, s_right_motor_command,
                           DEFAULT_MAX_PWM);
            } else if (tac_motors_enabled()) {
                s_left_motor_command = motor_command_apply_feedback(
                    tac_target_left(), s_vl_mm_s, s_left_motor_command,
                    DEFAULT_MAX_SPEED_MM_S, MOTOR_LEFT_PID_KP,
                    MOTOR_LEFT_PID_KI, &s_left_motor_integral);
                s_right_motor_command = motor_command_apply_feedback(
                    tac_target_right(), s_vr_mm_s, s_right_motor_command,
                    DEFAULT_MAX_SPEED_MM_S, MOTOR_RIGHT_PID_KP,
                    MOTOR_RIGHT_PID_KI, &s_right_motor_integral);
                motors_set(s_left_motor_command, s_right_motor_command,
                           DEFAULT_MAX_PWM);
            } else {
                s_left_motor_command = 0;
                s_right_motor_command = 0;
                s_left_motor_integral = 0.0f;
                s_right_motor_integral = 0.0f;
                motors_stop();
            }
        }
    }
}
