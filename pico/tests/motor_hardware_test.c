/*
 * Standalone physical motor bring-up test for the Wanderer Pico firmware.
 *
 * Purpose
 * -------
 * This test verifies behavior that the host-side test_motor_output program
 * cannot check: Pico GPIO/PWM operation, motor-driver wiring, wheel assignment,
 * and the physical forward/reverse direction of each motor. This is test
 * firmware, not the normal robot firmware.
 *
 * Build
 * -----
 * First configure the main Pico build as described in pico/README.md. Then,
 * from the repository root, build this firmware target:
 *
 *     cmake --build pico/build --target wanderer_motor_test
 *
 * The file to flash is:
 *
 *     pico/build/wanderer_motor_test.uf2
 *
 * Prepare the robot
 * -----------------
 * 1. Turn off motor power before handling the robot or changing wiring.
 * 2. Raise and securely support the chassis so both wheels can rotate freely.
 *    Do not run this test with the drive wheels touching the floor.
 * 3. Keep hands, cables, tools, and loose clothing clear of the wheels.
 * 4. Be ready to remove motor power immediately if a wheel binds, the chassis
 *    moves, or the observed sequence differs from the sequence below.
 *
 * Flash and run
 * -------------
 * 1. Hold BOOTSEL while connecting/resetting the Pico so it appears as a USB
 *    mass-storage device.
 * 2. Copy wanderer_motor_test.uf2 to the Pico.
 * 3. After the Pico reboots, open its USB CDC serial port in a terminal. No
 *    USB-to-UART adapter is required. A 115200-baud terminal setting is fine;
 *    USB CDC does not use a physical UART baud rate.
 * 4. Confirm again that the chassis is secure, then enable motor power.
 * 5. Type S in the USB serial terminal to arm the test. The sequence does not
 *    start merely because time has elapsed.
 * 6. After arming, there is a final five-second warning delay before movement.
 *
 * The firmware waits for the computer to open the USB serial port before it
 * prints the prompt. Closing and reopening the terminal does not start the
 * test; an explicit S character is still required.
 *
 * The MDD10A has no motor-supply status output connected to the Pico. Firmware
 * cannot electrically prove that motor power is present, so the S command is
 * the operator's confirmation that power is on. Automatic detection would
 * require a properly scaled, protected motor-voltage sense input.
 *
 * Expected sequence
 * -----------------
 * Each movement uses 40% PWM for one second, followed by one stopped second:
 *
 *     1. Left wheel forward
 *     2. Left wheel reverse
 *     3. Right wheel forward
 *     4. Right wheel reverse
 *     5. Both wheels forward
 *     6. Both wheels reverse
 *
 * The test passes only if the correct wheel moves in every single-wheel step,
 * forward/reverse match the robot's intended directions, both wheels behave
 * consistently, encoder counts change only for moving wheels, and the wheels
 * stop during every pause. Forward should increase ticks and reverse should
 * decrease them; adjust ENC_LEFT_SIGN or ENC_RIGHT_SIGN in config.h if needed.
 *
 * After the sequence, the firmware leaves both outputs stopped and waits for
 * another S command. Each command runs exactly one complete sequence. Remove
 * motor power, then flash pico/build/wanderer_pico.uf2 to restore the normal
 * robot firmware.
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#include "encoders.h"
#include "motor_output.h"
#include "motors.h"

/*
 * PWM values use per-mille units: 1000 is 100% duty, so 400 is 40%.
 * Keep this low enough for a safe bench test while still overcoming the
 * motors' starting friction.
 */
#define TEST_MAX_DUTY_PER_MILLE 400u
#define START_DELAY_MS 5000
#define RUN_MS 1000
#define PAUSE_MS 1000
#define ENCODER_REPORT_MS 100

_Static_assert(RUN_MS % ENCODER_REPORT_MS == 0,
               "RUN_MS must be divisible by ENCODER_REPORT_MS");

/*
 * USB enumeration alone does not mean a terminal is ready. Wait until the host
 * opens the CDC serial port so the operator sees the complete safety prompt.
 * Motor PWM has already been initialized to zero before this function runs.
 */
static void wait_for_usb_serial(void) {
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }

    /* Give the terminal time to finish opening before sending the first text. */
    sleep_ms(100);
}

/*
 * Keep the outputs stopped until the operator confirms that motor power is on.
 * A specific character is required so terminal startup noise or a leftover
 * newline cannot accidentally start the motors.
 */
static void wait_for_operator_arm(void) {
    printf("Motor outputs are stopped.\n");
    printf("Raise and secure the robot, enable motor power, then type S to start.\n");

    while (true) {
        int input = getchar_timeout_us(0);
        if (input == 's' || input == 'S') {
            printf("Start command received.\n");
            return;
        }
        tight_loop_contents();
    }
}

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

    absolute_time_t next_report = make_timeout_time_ms(ENCODER_REPORT_MS);
    for (uint32_t elapsed_ms = ENCODER_REPORT_MS;
         elapsed_ms <= RUN_MS;
         elapsed_ms += ENCODER_REPORT_MS) {
        sleep_until(next_report);
        next_report = delayed_by_ms(next_report, ENCODER_REPORT_MS);

        encoder_sample_t sample = encoders_sample();
        printf("  %4lu ms: L=%ld (%+ld), R=%ld (%+ld)\n",
               (unsigned long)elapsed_ms,
               (long)sample.left_ticks, (long)sample.left_delta,
               (long)sample.right_ticks, (long)sample.right_delta);
    }

    motors_stop();
    sleep_ms(PAUSE_MS);

    encoder_sample_t stopped = encoders_sample();
    printf("  stopped: L=%ld, R=%ld\n",
           (long)stopped.left_ticks, (long)stopped.right_ticks);
}

static void run_test_sequence(void) {
    encoders_reset();
    printf("Encoder counts reset to zero.\n");

    run_step("LEFT forward", +1, 0);
    run_step("LEFT reverse", -1, 0);
    run_step("RIGHT forward", 0, +1);
    run_step("RIGHT reverse", 0, -1);
    run_step("BOTH forward", +1, +1);
    run_step("BOTH reverse", -1, -1);
}

int main(void) {
    stdio_init_all();

    /*
     * Initialize immediately so PWM is held low even while motor power is off.
     * It is safe and desirable to establish the stopped logic state before the
     * MDD10A motor supply is switched on.
     */
    motors_init();
    encoders_init();

    wait_for_usb_serial();
    printf("\nWanderer motor hardware test\n");

    while (true) {
        /*
         * Every iteration begins stopped. After arming, allow time to remove
         * motor power before the first movement if the command was accidental.
         */
        motors_stop();
        wait_for_operator_arm();

        motors_stop();
        printf("Test starts in 5 seconds. Remove motor power now to abort.\n");
        sleep_ms(START_DELAY_MS);

        run_test_sequence();

        motors_stop();
        printf("Test complete; outputs are stopped.\n\n");
    }
}
