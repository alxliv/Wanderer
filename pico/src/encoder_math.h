#ifndef WANDERER_ENCODER_MATH_H
#define WANDERER_ENCODER_MATH_H

#include <stdint.h>

/*
 * Convert a tick count measured over elapsed_us into signed wheel velocity.
 * ticks_per_meter must use the same x4 quadrature-edge count as encoders.c.
 */
int16_t encoder_velocity_mm_s(int32_t tick_delta, float ticks_per_meter,
                              uint32_t elapsed_us);

#endif /* WANDERER_ENCODER_MATH_H */
