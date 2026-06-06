#ifndef WANDERER_MOTORS_H
#define WANDERER_MOTORS_H

#include <stdint.h>

void motors_init(void);
void motors_set(int16_t left_command, int16_t right_command, uint16_t max_pwm);
void motors_stop(void);

#endif /* WANDERER_MOTORS_H */
