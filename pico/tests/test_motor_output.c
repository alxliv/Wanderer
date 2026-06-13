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

    puts("motor output tests passed");
    return 0;
}
