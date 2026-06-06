#include <stdio.h>

#include "pico/stdlib.h"

#include "motor_output.h"
#include "motors.h"

#define TEST_DUTY 300
#define START_DELAY_MS 5000
#define RUN_MS 1000
#define PAUSE_MS 1000

static void run_step(const char *name, int16_t left, int16_t right) {
    printf("%s\n", name);
    motors_set(left, right, MOTOR_PWM_FULL_SCALE);
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

    run_step("LEFT forward", TEST_DUTY, 0);
    run_step("LEFT reverse", -TEST_DUTY, 0);
    run_step("RIGHT forward", 0, TEST_DUTY);
    run_step("RIGHT reverse", 0, -TEST_DUTY);
    run_step("BOTH forward", TEST_DUTY, TEST_DUTY);
    run_step("BOTH reverse", -TEST_DUTY, -TEST_DUTY);

    motors_stop();
    printf("Test complete; outputs are stopped.\n");
    while (true) {
        tight_loop_contents();
    }
}
