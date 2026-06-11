#include "motors.h"

#include "config.h"
#include "motor_output.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define MOTOR_PWM_HZ 20000u
#define MOTOR_PWM_WRAP (MOTOR_PWM_FULL_SCALE - 1u)

typedef struct {
    uint pwm_pin;
    uint dir_pin;
    bool current_direction;
} motor_channel_t;

static motor_channel_t left_motor = {
    .pwm_pin = M1_PWM_PIN,
    .dir_pin = M1_DIR_PIN,
};

static motor_channel_t right_motor = {
    .pwm_pin = M2_PWM_PIN,
    .dir_pin = M2_DIR_PIN,
};

static void init_channel(motor_channel_t *motor) {
    gpio_init(motor->dir_pin);
    gpio_set_dir(motor->dir_pin, GPIO_OUT);
    gpio_put(motor->dir_pin, false);

    gpio_init(motor->pwm_pin);
    gpio_set_dir(motor->pwm_pin, GPIO_OUT);
    gpio_put(motor->pwm_pin, false);
    gpio_set_function(motor->pwm_pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(motor->pwm_pin);

    pwm_config config = pwm_get_default_config();
    float divider = (float)clock_get_hz(clk_sys) /
                    ((float)MOTOR_PWM_HZ * (float)(MOTOR_PWM_WRAP + 1u));
    pwm_config_set_clkdiv(&config, divider);
    pwm_config_set_wrap(&config, MOTOR_PWM_WRAP);
    pwm_init(slice, &config, false);
    pwm_set_gpio_level(motor->pwm_pin, 0);
    pwm_set_enabled(slice, true);
    motor->current_direction = false;
}

static void apply_output(motor_channel_t *motor, motor_output_t output) {
    if (output.direction != motor->current_direction) {
        /* Remove drive before changing direction to avoid a reverse pulse. */
        pwm_set_gpio_level(motor->pwm_pin, 0);
        gpio_put(motor->dir_pin, output.direction);
        motor->current_direction = output.direction;
    }
    pwm_set_gpio_level(motor->pwm_pin, output.duty);
}

void motors_init(void) {
    init_channel(&left_motor);
    init_channel(&right_motor);
    motors_stop();
}

void motors_set(int16_t left_command, int16_t right_command, uint16_t max_pwm) {
    apply_output(&left_motor, motor_output_from_command(left_command, max_pwm));
    apply_output(&right_motor, motor_output_from_command(right_command, max_pwm));
}

void motors_stop(void) {
    motor_output_t stopped = {0};
    apply_output(&left_motor, stopped);
    apply_output(&right_motor, stopped);
}
