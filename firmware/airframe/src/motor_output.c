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
