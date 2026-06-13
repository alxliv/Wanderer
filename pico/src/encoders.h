#ifndef WANDERER_ENCODERS_H
#define WANDERER_ENCODERS_H

#include <stdint.h>

typedef struct {
    int32_t left_ticks;
    int32_t right_ticks;
    int32_t left_delta;
    int32_t right_delta;
} encoder_sample_t;

/*
 * Start two PIO state machines that decode every valid A/B transition.
 * The A signal must be on each configured base pin and B on base pin + 1.
 */
void encoders_init(void);

/* Make the current physical positions the zero point for subsequent samples. */
void encoders_reset(void);

/*
 * Return cumulative ticks since reset plus ticks since the previous sample.
 * Counts use wrapping 32-bit arithmetic, matching the protocol's i32 fields.
 */
encoder_sample_t encoders_sample(void);

#endif /* WANDERER_ENCODERS_H */
