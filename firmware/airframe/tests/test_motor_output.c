#include <assert.h>
#include <limits.h>
#include <stdio.h>

#include "motor_output.h"

/*
 * This host-side unit test verifies the pure conversion between a signed motor
 * command and the MDD10A's DIR/PWM outputs. It does not access Pico hardware.
 *
 * Keeping this logic testable on the development computer is important because
 * a sign error can reverse a wheel, while a clamp error can apply more power
 * than the configured safety limit. The separate motor_hardware_test confirms
 * actual GPIO, wiring, and motor operation after this arithmetic is validated.
 */

/* Check both output fields so an incorrect direction cannot pass just because
 * the PWM duty happens to be correct, or vice versa. */
static void expect_output(int16_t command, uint16_t limit,
                          bool direction, uint16_t duty) {
    motor_output_t output = motor_output_from_command(command, limit);
    assert(output.direction == direction);
    assert(output.duty == duty);
}

int main(void) {
    /* Per-wheel gain is fixed-point per-mille and symmetric by direction. */
    assert(motor_command_apply_gain(250, 1109) == 277);
    assert(motor_command_apply_gain(-250, 1109) == -277);
    assert(motor_command_apply_gain(250, 902) == 226);
    assert(motor_command_apply_gain(-250, 902) == -226);
    assert(motor_command_apply_gain(250, 1000) == 250);
    assert(motor_command_apply_gain(1000, 1109) == 1000);
    assert(motor_command_apply_gain(-1000, 1109) == -1000);
    assert(motor_command_apply_gain(250, 0) == 0);

    /* Deadband feed-forward preserves a true stop and full-scale endpoint. */
    assert(motor_command_apply_deadband(0, 80) == 0);
    assert(motor_command_apply_deadband(1, 80) == 80);
    assert(motor_command_apply_deadband(-1, 80) == -80);
    assert(motor_command_apply_deadband(500, 80) == 540);
    assert(motor_command_apply_deadband(-500, 80) == -540);
    assert(motor_command_apply_deadband(1000, 80) == 1000);
    assert(motor_command_apply_deadband(-1000, 80) == -1000);
    assert(motor_command_apply_deadband(500, 0) == 500);
    assert(motor_command_apply_deadband(INT16_MIN, 80) == -1000);
    assert(motor_command_apply_deadband(1, 2000) == 1000);

    /* Zero command is a defined stop state. */
    expect_output(0, 1000, false, 0);

    /* Command sign selects direction; magnitude becomes PWM duty. */
    expect_output(500, 1000, false, 500);
    expect_output(-500, 1000, true, 500);

    /* Commands outside the per-mille range are limited to 1000 (100%). */
    expect_output(1200, 1000, false, 1000);
    expect_output(-1200, 1000, true, 1000);

    /* A configured limit must cap both forward and reverse output equally. */
    expect_output(800, 350, false, 350);
    expect_output(-800, 350, true, 350);

    /* A zero limit disables output even when a non-zero command is present. */
    expect_output(500, 0, false, 0);

    /* INT16_MIN exercises safe absolute-value handling at the signed boundary. */
    expect_output(INT16_MIN, 1000, true, 1000);

    /* An invalid limit above full scale must not increase the command duty. */
    expect_output(500, 2000, false, 500);

    /* PI output is feed-forward plus a correction, not accumulated output. */
    float integral = 0.0f;
    assert(motor_command_apply_feedback(300, 300, 600, 0.01f,
                                        0.5f, 0.1f, &integral) == 500);
    assert(integral == 0.0f);

    assert(motor_command_apply_feedback(300, 200, 600, 0.01f,
                                        0.5f, 0.1f, &integral) == 550);
    assert(integral == 1.0f);

    /* Forward and reverse corrections are symmetric. */
    integral = 0.0f;
    assert(motor_command_apply_feedback(-300, -200, 600, 0.01f,
                                        0.5f, 0.1f, &integral) == -550);
    assert(integral == -1.0f);

    /* A stop never applies reverse braking and forgets retained error. */
    integral = 123.0f;
    assert(motor_command_apply_feedback(0, 600, 600, 0.01f,
                                        0.5f, 0.1f, &integral) == 0);
    assert(integral == 0.0f);

    /* Saturation must not wind the integral farther into the limit. */
    integral = 0.0f;
    assert(motor_command_apply_feedback(600, 0, 600, 0.01f,
                                        0.5f, 0.1f, &integral) == 1000);
    assert(integral == 0.0f);

    /* Integral state is elapsed-time based and has a finite bound. */
    integral = 0.0f;
    assert(motor_command_apply_feedback(100, 0, 600, 30.0f,
                                        0.0f, 0.1f, &integral) == 366);
    assert(integral == 2000.0f);

    /* A motor-like lag settles without a command reversal. */
    float speed_mm_s = 0.0f;
    int16_t command = 0;
    integral = 0.0f;
    for (int tick = 0; tick < 200; ++tick) {
        command = motor_command_apply_feedback(250, (int16_t)speed_mm_s,
                                               600, 0.01f,
                                               0.290f, 0.058f, &integral);
        assert(command >= 0);
        const float demanded_speed = 0.6f * command;
        speed_mm_s += (demanded_speed - speed_mm_s) * 0.1f;
    }
    assert(speed_mm_s > 245.0f && speed_mm_s < 255.0f);
    assert(motor_command_apply_feedback(0, (int16_t)speed_mm_s,
                                        600, 0.01f,
                                        0.290f, 0.058f, &integral) == 0);
    assert(integral == 0.0f);

    puts("motor output tests passed");
    return 0;
}
