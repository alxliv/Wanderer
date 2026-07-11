#include "encoders.h"

#include <string.h>

#include "hardware/pio.h"

#include "config.h"
#include "quadrature_encoder.pio.h"

#define ENCODER_PIO pio0
#define LEFT_ENCODER_SM 0u
#define RIGHT_ENCODER_SM 1u

static struct {
    uint32_t left_zero;
    uint32_t right_zero;
    uint32_t previous_left;
    uint32_t previous_right;
} state;

static int32_t bits_to_i32(uint32_t value) {
    int32_t result;
    memcpy(&result, &value, sizeof result);
    return result;
}

static uint32_t apply_sign(uint32_t value, int sign) {
    return sign < 0 ? (0u - value) : value;
}

static uint32_t read_left_ticks(void) {
    uint32_t raw = quadrature_encoder_get_count(ENCODER_PIO, LEFT_ENCODER_SM);
    return apply_sign(raw - state.left_zero, ENC_LEFT_SIGN);
}

static uint32_t read_right_ticks(void) {
    uint32_t raw = quadrature_encoder_get_count(ENCODER_PIO, RIGHT_ENCODER_SM);
    return apply_sign(raw - state.right_zero, ENC_RIGHT_SIGN);
}

void encoders_init(void) {
    pio_add_program(ENCODER_PIO, &quadrature_encoder_program);
    quadrature_encoder_program_init(ENCODER_PIO, LEFT_ENCODER_SM,
                                    ENC_LEFT_PIN_BASE);
    quadrature_encoder_program_init(ENCODER_PIO, RIGHT_ENCODER_SM,
                                    ENC_RIGHT_PIN_BASE);
    encoders_reset();
}

void encoders_reset(void) {
    state.left_zero =
        quadrature_encoder_get_count(ENCODER_PIO, LEFT_ENCODER_SM);
    state.right_zero =
        quadrature_encoder_get_count(ENCODER_PIO, RIGHT_ENCODER_SM);
    state.previous_left = 0;
    state.previous_right = 0;
}

encoder_sample_t encoders_sample(void) {
    uint32_t left = read_left_ticks();
    uint32_t right = read_right_ticks();

    encoder_sample_t sample = {
        .left_ticks = bits_to_i32(left),
        .right_ticks = bits_to_i32(right),
        .left_delta = bits_to_i32(left - state.previous_left),
        .right_delta = bits_to_i32(right - state.previous_right),
    };

    state.previous_left = left;
    state.previous_right = right;
    return sample;
}
