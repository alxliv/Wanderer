#include <assert.h>
#include <limits.h>
#include <stdio.h>

#include "encoder_math.h"

int main(void) {
    /* 100 ticks in 10 ms at 10,000 ticks/meter is exactly 1,000 mm/s. */
    assert(encoder_velocity_mm_s(100, 10000.0f, 10000) == 1000);
    assert(encoder_velocity_mm_s(-100, 10000.0f, 10000) == -1000);

    /* Fractional results round to the nearest signed whole mm/s. */
    assert(encoder_velocity_mm_s(1, 3000.0f, 10000) == 33);
    assert(encoder_velocity_mm_s(-1, 3000.0f, 10000) == -33);

    /* Invalid calibration/timing cannot produce meaningful velocity. */
    assert(encoder_velocity_mm_s(100, 0.0f, 10000) == 0);
    assert(encoder_velocity_mm_s(100, -1.0f, 10000) == 0);
    assert(encoder_velocity_mm_s(100, 10000.0f, 0) == 0);

    /* Protocol telemetry is i16, so extreme measurements must saturate. */
    assert(encoder_velocity_mm_s(INT32_MAX, 1.0f, 1) == INT16_MAX);
    assert(encoder_velocity_mm_s(INT32_MIN, 1.0f, 1) == INT16_MIN);

    puts("encoder math tests passed");
    return 0;
}
