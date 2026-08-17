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
                                     int16_t current_command,
                                     int16_t max_speed_mm_s,
                                     float kp, float ki, float *integral) {
    if (max_speed_mm_s <= 0) {
        return current_command;
    }

    int32_t target_permille = ((int32_t)target_mm_s * 1000) / max_speed_mm_s;
    int32_t measured_permille = ((int32_t)measured_mm_s * 1000) / max_speed_mm_s;
    int32_t error_permille = target_permille - measured_permille;

    if (integral) {
        *integral += (float)error_permille;
        if (*integral > 2000.0f) {
            *integral = 2000.0f;
        } else if (*integral < -2000.0f) {
            *integral = -2000.0f;
        }
    }

    float correction = (float)error_permille * kp;
    if (integral) {
        correction += (*integral) * ki;
    }

    int32_t adjusted = (int32_t)current_command + (int32_t)correction;
    if (adjusted > 1000) {
        adjusted = 1000;
    } else if (adjusted < -1000) {
        adjusted = -1000;
    }
    return (int16_t)adjusted;
}
