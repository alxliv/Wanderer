#ifndef WANDERER_MOTOR_OUTPUT_H
#define WANDERER_MOTOR_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_PWM_FULL_SCALE 1000u

typedef struct {
    bool direction;
    uint16_t duty;
} motor_output_t;

/* Apply a per-mille gain with symmetric rounding and clamp to full scale. */
int16_t motor_command_apply_gain(int16_t command, uint16_t gain_permille);

/*
 * Map a non-zero logical command onto the motor's moving range. Zero remains
 * an actual stop; full-scale remains full-scale. Apply this before per-wheel
 * gain so a deadband measured through motors_set() retains its meaning.
 */
int16_t motor_command_apply_deadband(int16_t command, uint16_t deadband);

/*
 * Convert a signed per-mille command into an MDD10A DIR and PWM output.
 * Positive commands use DIR=0; negative commands use DIR=1.
 * A zero command or zero limit stops the channel with PWM=0 and DIR=0.
 */
motor_output_t motor_output_from_command(int16_t command, uint16_t max_pwm);

/*
 * Apply positional wheel-speed PI around the open-loop speed-to-duty mapping.
 * kp converts instantaneous speed error in mm/s to per-mille duty. ki converts
 * time-integrated speed error to per-mille duty; dt_s is the sample interval.
 * A zero target is an exact stop and clears the integral accumulator.
 */
int16_t motor_command_apply_feedback(int16_t target_mm_s,
                                     int16_t measured_mm_s,
                                     int16_t max_speed_mm_s,
                                     float dt_s, float kp, float ki,
                                     float *integral);

#ifdef __cplusplus
}
#endif

#endif /* WANDERER_MOTOR_OUTPUT_H */
