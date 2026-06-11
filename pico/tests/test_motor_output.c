#include <assert.h>
#include <limits.h>
#include <stdio.h>

#include "motor_output.h"

static void expect_output(int16_t command, uint16_t limit,
                          bool direction, uint16_t duty) {
    motor_output_t output = motor_output_from_command(command, limit);
    assert(output.direction == direction);
    assert(output.duty == duty);
}

int main(void) {
    expect_output(0, 1000, false, 0);
    expect_output(500, 1000, false, 500);
    expect_output(-500, 1000, true, 500);
    expect_output(1200, 1000, false, 1000);
    expect_output(-1200, 1000, true, 1000);
    expect_output(800, 350, false, 350);
    expect_output(-800, 350, true, 350);
    expect_output(500, 0, false, 0);
    expect_output(INT16_MIN, 1000, true, 1000);
    expect_output(500, 2000, false, 500);

    puts("motor output tests passed");
    return 0;
}
