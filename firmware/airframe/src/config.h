/*
 * Wanderer airframe (Pico 2 / RP2350) - pin map & tunable defaults.
 *
 * Pin assignments are the firmware's source of truth; keep hardware/wiring.md
 * in sync with this file.
 */
#ifndef WANDERER_CONFIG_H
#define WANDERER_CONFIG_H

/* ---- I2C0: master to VL53L0X ToF (wired in the ToF step) ---- */
#define TOF_SDA_PIN       4     /* GP4  (I2C0 SDA) */
#define TOF_SCL_PIN       5     /* GP5  (I2C0 SCL) */
#define TOF_XSHUT_PIN     8     /* GP8  (optional reset/boot control, O5) */

/* ---- Cytron MDD10A channel 1 (left) ---- */
#define M1_PWM_PIN        16    /* PWM1 (speed)      */
#define M1_DIR_PIN        17    /* DIR1 (direction)  */

/* ---- Cytron MDD10A channel 2 (right) ---- */
#define M2_PWM_PIN        19    /* PWM2 (speed)      */
#define M2_DIR_PIN        20    /* DIR2 (direction)  */

/* ---- Motor polarity ----
 * A POSITIVE command must drive the vehicle toward bearing 0 (forward). See
 * docs/Wanderer_Command_Architecture.md section 2a for the body frame. If a
 * wheel spins the wrong way, set its sign to -1 here rather than re-wiring;
 * `python tools/backdoor.py --calibrate` asks about each wheel in turn and
 * tells you which one to change.
 */
#define MOTOR_LEFT_SIGN    1
#define MOTOR_RIGHT_SIGN   1

/* Motor DEADBAND */
/* Measured 2026-08-12 with tools/backdoor.py, wheels
     raised and UNLOADED -- see the caveat below. */
#define MOTOR_DEADBAND_LEFT  55
#define MOTOR_DEADBAND_RIGHT 35
/* Lowest duty at which BOTH wheels reliably turn: 65. Commands
     below this move nothing; feed-forward past it or refuse them. */


/* ---- Quadrature encoders (3.3 V logic; decoded via PIO) ----
 * Each encoder uses two CONSECUTIVE GPIO (A on base pin, B on base+1) so a
 * single PIO state machine can read both channels.
 *
 * The signs normalize counts so that motion toward bearing 0 (straight ahead
 * from the driver seat -- docs/Wanderer_Command_Architecture.md section 2a)
 * counts UP. encoders_sample() applies them, so every count above this line
 * is already corrected; a bench tool reading rising counts has learned that
 * the sign here is right, NOT that the sign is +1.
 *
 * Verified by `python tools/backdoor.py --calibrate`, which names the one to
 * change if either is inverted.
 */
#define ENC_LEFT_PIN_BASE  10    /* left:  GP10 (A), GP11 (B) */
#define ENC_RIGHT_PIN_BASE 12    /* right: GP12 (A), GP13 (B) */
#define ENC_LEFT_SIGN      -1
#define ENC_RIGHT_SIGN      1

/* ---- Pico2 UART (UART0) -- the flight interface to the RPI5 UART ----
 * Raw line protocol per protocol/cockpit_protocol.md -- NOT stdio. stdio
 * stays on USB CDC for bench logs. 115200 8N1 baseline (spec section 12).
 */
#define PICO2_UART        uart0
#define PICO2_UART_TX_PIN 0     /* GP0 (UART0 TX) */
#define PICO2_UART_RX_PIN 1     /* GP1 (UART0 RX) */
#define PICO2_UART_BAUD   115200

/* ---- Vehicle geometry ---- */
#define TRACK_WIDTH_M     0.30f /* wheel-to-wheel; calibrate on the rover */

/* ---- Control loop ---- */
#define CONTROL_HZ        100   /* reflexive control-loop rate */

/* ---- Tunable defaults ---- */
#define DEFAULT_TICKS_PER_METER  10000.0f /* x4 edge count; calibrate */
#define DEFAULT_PID_KP           0.5f
#define DEFAULT_PID_KI           0.1f
#define DEFAULT_PID_KD           0.0f
#define DEFAULT_MAX_PWM          1000      /* per-mille clamp (0..1000) */
#define DEFAULT_MAX_SPEED_MM_S   600       /* open-loop full-scale; calibrate */
#define DEFAULT_WATCHDOG_10MS    50        /* 50 * 10 ms = 500 ms */
#define DEFAULT_OBSTACLE_STOP_MM 0         /* disabled until ToF is wired */

/* ---- Firmware version (reported in INFO registers) ---- */
#define FW_VERSION_MAJOR  0
#define FW_VERSION_MINOR  4

#endif /* WANDERER_CONFIG_H */
