#ifndef WANDERER_ANGLE_H
#define WANDERER_ANGLE_H

#include <math.h>

/* Wrap radians into (-pi, pi]. Kept header-only for C and C++ firmware code. */
static inline float wrap_pi(float angle)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 2.0f * pi;
    const float reduced = fmodf(angle + pi, two_pi);
    const float wrapped = (reduced <= 0.0f) ? reduced + pi : reduced - pi;
    // A float expression such as -3.0f * pi can land a few ulps above -pi.
    // Preserve this API's closed upper boundary despite that rounding.
    return (fabsf(wrapped + pi) < 1e-5f) ? pi : wrapped;
}

#endif /* WANDERER_ANGLE_H */
