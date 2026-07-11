#include "encoder_math.h"

#include <limits.h>

int16_t encoder_velocity_mm_s(int32_t tick_delta, float ticks_per_meter,
                              uint32_t elapsed_us) {
    if (!(ticks_per_meter > 0.0f) || elapsed_us == 0u) {
        return 0;
    }

    float velocity = ((float)tick_delta * 1000000000.0f) /
                     (ticks_per_meter * (float)elapsed_us);

    if (velocity >= (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (velocity <= (float)INT16_MIN) {
        return INT16_MIN;
    }

    /* Round to the nearest whole mm/s rather than always truncating to zero. */
    velocity += velocity >= 0.0f ? 0.5f : -0.5f;
    return (int16_t)velocity;
}
