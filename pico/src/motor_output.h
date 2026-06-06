#ifndef WANDERER_MOTOR_OUTPUT_H
#define WANDERER_MOTOR_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PWM_FULL_SCALE 1000u

typedef struct {
    bool in_a;
    bool in_b;
    uint16_t duty;
} motor_output_t;

/*
 * Convert a signed per-mille command into an L298N direction and PWM output.
 * Positive commands use IN_A=1/IN_B=0; negative commands reverse that pair.
 * A zero command or zero limit coasts with both direction inputs low.
 */
motor_output_t motor_output_from_command(int16_t command, uint16_t max_pwm);

#endif /* WANDERER_MOTOR_OUTPUT_H */
