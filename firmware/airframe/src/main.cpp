// Wanderer airframe main -- the tactical layer behind the cockpit UART.
//
// Wiring (all shared code, no logic of its own here):
//   UART0 bytes  -> cockpit_feed() -> cockpit_handler -> tac_* (FSM)
//   FSM change   -> `!fault`/`!state` lines -> UART0
//   tac targets  -> open-loop per-mille -> motors_set()
//   encoders     -> odometry provider -> `=ok get_odometry ...`
//
// Motor control is OPEN LOOP for now: wheel target mm/s maps linearly to
// per-mille PWM via DEFAULT_MAX_SPEED_MM_S. The closed velocity loop (PID
// constants already waiting in config.h) is its own upcoming step; nothing
// in the cockpit protocol changes when it lands.
//
// stdio stays on USB CDC for bench logs (`*` lines); the cockpit UART
// carries ONLY protocol lines.

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "cockpit_handler.h"
#include "tactical.h"
#include "config.h"
#include "encoders.h"
#include "encoder_math.h"
#include "motors.h"

static uint64_t now_us(void)
{
    return to_us_since_boot(get_absolute_time());
}

// ---- cockpit transport -----------------------------------------------------

static void cockpit_line_out(const char *line)
{
    uart_puts(COCKPIT_UART, line);
    uart_puts(COCKPIT_UART, "\r\n");
}

static void cockpit_uart_init(void)
{
    uart_init(COCKPIT_UART, COCKPIT_BAUD);
    gpio_set_function(COCKPIT_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(COCKPIT_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(COCKPIT_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(COCKPIT_UART, true);
}

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

    cockpit_uart_init();
    motors_init();
    encoders_init();
    encoders_reset();

    tac_init();
    cockpit_init(cockpit_line_out, FW_VERSION_MAJOR, FW_VERSION_MINOR,
                 TRACK_WIDTH_M / 2.0f);
    cockpit_set_odometry_provider(odometry_provider);
    // Relay sink deliberately not set: `^` payloads are dropped until the
    // RF modem hat lands (cockpit spec section 4).

    printf("*airframe fw %u.%u cockpit on uart0 @%u\r\n",
           FW_VERSION_MAJOR, FW_VERSION_MINOR, (unsigned)COCKPIT_BAUD);

    const uint32_t control_period_us = 1000000u / CONTROL_HZ;
    uint64_t next_control = now_us();

    while (true) {
        // Pump every waiting cockpit byte; time-stamp at arrival.
        while (uart_is_readable(COCKPIT_UART))
            cockpit_feed((char)uart_getc(COCKPIT_UART), now_us());

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

            // FSM housekeeping: deadman and fallback ramp.
            tac_tick(t);

            // Targets -> wheels. SAFE/FAULT gate to zero via motors_stop so
            // the driver's outputs are unambiguous, not merely zero-valued.
            if (tac_motors_enabled())
                motors_set(permille_from_mm_s(tac_target_left()),
                           permille_from_mm_s(tac_target_right()),
                           DEFAULT_MAX_PWM);
            else
                motors_stop();
        }
    }
}
