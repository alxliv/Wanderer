#ifndef WANDERER_MOTOR_OUTPUT_H
#define WANDERER_MOTOR_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PWM_FULL_SCALE 1000u

typedef struct {
    bool direction;
    uint16_t duty;
} motor_output_t;

/*
 * Convert a signed per-mille command into an MDD10A DIR and PWM output.
 * Positive commands use DIR=0; negative commands use DIR=1.
 * A zero command or zero limit stops the channel with PWM=0 and DIR=0.
 */
motor_output_t motor_output_from_command(int16_t command, uint16_t max_pwm);

#endif /* WANDERER_MOTOR_OUTPUT_H */
