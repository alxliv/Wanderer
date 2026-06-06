#include "motor_output.h"

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

    output.in_a = command > 0;
    output.in_b = command < 0;
    output.duty = (uint16_t)magnitude;
    return output;
}
