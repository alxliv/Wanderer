#include "motors.h"

#include "config.h"
#include "motor_output.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define MOTOR_PWM_HZ 20000u
#define MOTOR_PWM_WRAP (MOTOR_PWM_FULL_SCALE - 1u)

typedef struct {
    uint enable_pin;
    uint in_a_pin;
    uint in_b_pin;
    bool current_in_a;
    bool current_in_b;
} motor_channel_t;

static motor_channel_t left_motor = {
    .enable_pin = M_A_ENA_PIN,
    .in_a_pin = M_A_IN1_PIN,
    .in_b_pin = M_A_IN2_PIN,
};

static motor_channel_t right_motor = {
    .enable_pin = M_B_ENB_PIN,
    .in_a_pin = M_B_IN3_PIN,
    .in_b_pin = M_B_IN4_PIN,
};

static void init_channel(motor_channel_t *motor) {
    gpio_init(motor->in_a_pin);
    gpio_set_dir(motor->in_a_pin, GPIO_OUT);
    gpio_put(motor->in_a_pin, false);

    gpio_init(motor->in_b_pin);
    gpio_set_dir(motor->in_b_pin, GPIO_OUT);
    gpio_put(motor->in_b_pin, false);

    gpio_set_function(motor->enable_pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(motor->enable_pin);

    pwm_config config = pwm_get_default_config();
    float divider = (float)clock_get_hz(clk_sys) /
                    ((float)MOTOR_PWM_HZ * (float)(MOTOR_PWM_WRAP + 1u));
    pwm_config_set_clkdiv(&config, divider);
    pwm_config_set_wrap(&config, MOTOR_PWM_WRAP);
    pwm_init(slice, &config, true);
    pwm_set_gpio_level(motor->enable_pin, 0);
    motor->current_in_a = false;
    motor->current_in_b = false;
}

static void apply_output(motor_channel_t *motor, motor_output_t output) {
    if (output.in_a != motor->current_in_a ||
        output.in_b != motor->current_in_b) {
        /* Remove drive before changing direction to avoid a reverse pulse. */
        pwm_set_gpio_level(motor->enable_pin, 0);
        gpio_put(motor->in_a_pin, output.in_a);
        gpio_put(motor->in_b_pin, output.in_b);
        motor->current_in_a = output.in_a;
        motor->current_in_b = output.in_b;
    }
    pwm_set_gpio_level(motor->enable_pin, output.duty);
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
    motor_output_t coast = {0};
    apply_output(&left_motor, coast);
    apply_output(&right_motor, coast);
}
