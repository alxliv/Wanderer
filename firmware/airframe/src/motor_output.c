#include "motor_output.h"

int16_t motor_command_apply_gain(int16_t command, uint16_t gain_permille) {
    int32_t scaled = (int32_t)command * gain_permille;

    /* Round magnitude to nearest while preserving symmetry around zero. */
    if (scaled > 0) {
        scaled = (scaled + 500) / 1000;
    } else if (scaled < 0) {
        scaled = (scaled - 500) / 1000;
    }

    if (scaled > (int32_t)MOTOR_PWM_FULL_SCALE) {
        scaled = MOTOR_PWM_FULL_SCALE;
    } else if (scaled < -(int32_t)MOTOR_PWM_FULL_SCALE) {
        scaled = -(int32_t)MOTOR_PWM_FULL_SCALE;
    }
    return (int16_t)scaled;
}

int16_t motor_command_apply_deadband(int16_t command, uint16_t deadband) {
    if (command == 0) {
        return 0;
    }
    if (deadband > MOTOR_PWM_FULL_SCALE) {
        deadband = MOTOR_PWM_FULL_SCALE;
    }

    int32_t magnitude = command;
    if (magnitude < 0) {
        magnitude = -magnitude;
    }
    if (magnitude > (int32_t)MOTOR_PWM_FULL_SCALE) {
        magnitude = MOTOR_PWM_FULL_SCALE;
    }

    magnitude = deadband +
                ((int32_t)(MOTOR_PWM_FULL_SCALE - deadband) * magnitude) /
                    MOTOR_PWM_FULL_SCALE;
    return (int16_t)(command < 0 ? -magnitude : magnitude);
}

motor_output_t motor_output_from_command(int16_t command, uint16_t max_pwm) {
    motor_output_t output = {0};

    if (max_pwm > MOTOR_PWM_FULL_SCALE) {
        max_pwm = MOTOR_PWM_FULL_SCALE;
    }
    if (command == 0 || max_pwm == 0) {
        return output;
    }

    int32_t magnitude = command;
    if (magnitude < 0) {
        magnitude = -magnitude;
    }
    if (magnitude > (int32_t)MOTOR_PWM_FULL_SCALE) {
        magnitude = MOTOR_PWM_FULL_SCALE;
    }
    if (magnitude > max_pwm) {
        magnitude = max_pwm;
    }

    output.direction = command < 0;
    output.duty = (uint16_t)magnitude;
    return output;
}

int16_t motor_command_apply_feedback(int16_t target_mm_s,
                                     int16_t measured_mm_s,
                                     int16_t max_speed_mm_s,
                                     float dt_s, float kp, float ki,
                                     float *integral) {
    if (max_speed_mm_s <= 0 || target_mm_s == 0) {
        if (integral) {
            *integral = 0.0f;
        }
        return 0;
    }

    const int32_t feedforward =
        ((int32_t)target_mm_s * 1000) / max_speed_mm_s;
    const int32_t error_mm_s = (int32_t)target_mm_s - measured_mm_s;
    const float old_integral = integral ? *integral : 0.0f;
    float next_integral = old_integral;

    if (integral && dt_s > 0.0f && ki > 0.0f) {
        next_integral += (float)error_mm_s * dt_s;
        if (next_integral > 2000.0f) {
            next_integral = 2000.0f;
        } else if (next_integral < -2000.0f) {
            next_integral = -2000.0f;
        }
    }

    float adjusted = (float)feedforward + (float)error_mm_s * kp
                   + next_integral * ki;

    /* Do not wind the integral farther into output saturation. */
    if ((adjusted > (float)MOTOR_PWM_FULL_SCALE && error_mm_s > 0)
            || (adjusted < -(float)MOTOR_PWM_FULL_SCALE && error_mm_s < 0)) {
        next_integral = old_integral;
        adjusted = (float)feedforward + (float)error_mm_s * kp
                 + next_integral * ki;
    }
    if (integral) {
        *integral = next_integral;
    }

    if (adjusted > (float)MOTOR_PWM_FULL_SCALE) {
        adjusted = (float)MOTOR_PWM_FULL_SCALE;
    } else if (adjusted < -(float)MOTOR_PWM_FULL_SCALE) {
        adjusted = -(float)MOTOR_PWM_FULL_SCALE;
    }
    return (int16_t)adjusted;
}
