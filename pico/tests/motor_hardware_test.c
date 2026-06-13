#include <stdio.h>

#include "pico/stdlib.h"

#include "motor_output.h"
#include "motors.h"

/*
 * PWM values use per-mille units: 1000 is 100% duty, so 300 is 30%.
 * Keep this low enough for a safe bench test while still overcoming the
 * motors' starting friction.
 */
#define TEST_MAX_DUTY_PER_MILLE 300u
#define START_DELAY_MS 5000
#define RUN_MS 1000
#define PAUSE_MS 1000

/*
 * A direction is -1 for reverse, 0 for stopped, or +1 for forward.
 * Full-scale commands are intentional here: TEST_MAX_DUTY_PER_MILLE is the
 * separate safety clamp that determines the actual PWM sent to each motor.
 */
static void run_step(const char *name, int8_t left_direction, int8_t right_direction) {
    int16_t left_command = (int16_t)(left_direction * (int16_t)MOTOR_PWM_FULL_SCALE);
    int16_t right_command = (int16_t)(right_direction * (int16_t)MOTOR_PWM_FULL_SCALE);

    printf("%s\n", name);
    motors_set(left_command, right_command, TEST_MAX_DUTY_PER_MILLE);
    sleep_ms(RUN_MS);
    motors_stop();
    sleep_ms(PAUSE_MS);
}

int main(void) {
    stdio_init_all();
    motors_init();

    printf("\nWanderer motor hardware test\n");
    printf("Raise the robot so both wheels turn freely. Test starts in 5 seconds.\n");
    sleep_ms(START_DELAY_MS);

    run_step("LEFT forward", +1, 0);
    run_step("LEFT reverse", -1, 0);
    run_step("RIGHT forward", 0, +1);
    run_step("RIGHT reverse", 0, -1);
    run_step("BOTH forward", +1, +1);
    run_step("BOTH reverse", -1, -1);

    motors_stop();
    printf("Test complete; outputs are stopped.\n");
    while (true) {
        tight_loop_contents();
    }
}
