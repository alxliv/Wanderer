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

/* ---- Quadrature encoders (3.3 V logic; decoded via PIO) ----
 * Each encoder uses two CONSECUTIVE GPIO (A on base pin, B on base+1) so a
 * single PIO state machine can read both channels. The sign values normalize
 * counts so a physically forward wheel should report positive ticks. Change a
 * sign to -1 if the motor hardware test reports negative ticks while forward.
 */
#define ENC_LEFT_PIN_BASE  10    /* left:  GP10 (A), GP11 (B) */
#define ENC_RIGHT_PIN_BASE 12    /* right: GP12 (A), GP13 (B) */
#define ENC_LEFT_SIGN      -1
#define ENC_RIGHT_SIGN      1

/* ---- Control loop ---- */
#define CONTROL_HZ        100   /* reflexive control-loop rate */

/* ---- Tunable defaults ---- */
#define DEFAULT_TICKS_PER_METER  10000.0f /* x4 edge count; calibrate */
#define DEFAULT_PID_KP           0.5f
#define DEFAULT_PID_KI           0.1f
#define DEFAULT_PID_KD           0.0f
#define DEFAULT_MAX_PWM          1000      /* per-mille clamp (0..1000) */
#define DEFAULT_WATCHDOG_10MS    50        /* 50 * 10 ms = 500 ms */
#define DEFAULT_OBSTACLE_STOP_MM 0         /* disabled until ToF is wired */

/* ---- Firmware version (reported in INFO registers) ---- */
#define FW_VERSION_MAJOR  0
#define FW_VERSION_MINOR  3

#endif /* WANDERER_CONFIG_H */
