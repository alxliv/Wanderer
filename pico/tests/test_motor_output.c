#include <assert.h>
#include <limits.h>
#include <stdio.h>

#include "motor_output.h"

static void expect_output(int16_t command, uint16_t limit,
                          bool in_a, bool in_b, uint16_t duty) {
    motor_output_t output = motor_output_from_command(command, limit);
    assert(output.in_a == in_a);
    assert(output.in_b == in_b);
    assert(output.duty == duty);
}

int main(void) {
    expect_output(0, 1000, false, false, 0);
    expect_output(500, 1000, true, false, 500);
    expect_output(-500, 1000, false, true, 500);
    expect_output(1200, 1000, true, false, 1000);
    expect_output(-1200, 1000, false, true, 1000);
    expect_output(800, 350, true, false, 350);
    expect_output(-800, 350, false, true, 350);
    expect_output(500, 0, false, false, 0);
    expect_output(INT16_MIN, 1000, false, true, 1000);
    expect_output(500, 2000, true, false, 500);

    puts("motor output tests passed");
    return 0;
}
