/*
 * Wanderer airframe (Pico 2 / RP2350) - pin map & tunable defaults.
 *
 * Pin assignments are the firmware's source of truth; keep hardware/wiring.md
 * in sync with this file.
 */
#ifndef WANDERER_CONFIG_H
#define WANDERER_CONFIG_H

#define PI 3.14159265358979323846f
#define DEG2RAD(x) (((x)*PI)/180)

/* ---- I2C0: master to VL53L0X ToF (wired in the ToF step) ---- */
#define TOF_SDA_PIN       4     /* GP4  (I2C0 SDA) */
#define TOF_SCL_PIN       5     /* GP5  (I2C0 SCL) */
#define TOF_XSHUT_PIN     8     /* GP8  (optional reset/boot control, O5) */

/* ---- I2C1: MinIMU-9 v6 (LSM6DSO gyro/accelerometer) ---- */
#define IMU_SDA_PIN       6     /* GP6  (I2C1 SDA) */
#define IMU_SCL_PIN       7     /* GP7  (I2C1 SCL) */
#define IMU_I2C_BAUD      400000
#define IMU_LSM6DSO_ADDR  0x6B  /* SA0 high on the MinIMU-9 v6 carrier */
/* Turning RIGHT must produce a positive yaw rate. Verify by hand in M1. */
#define IMU_YAW_SIGN      (1)
#define IMU_SCALE         1.0035291f  /* calibrated in M3 */
#define IMU_STALE_MS      50

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

/* ---- Open-loop motor matching ----
 * Raised-wheel measurement at 250 permille for 3 s gave 4600 left ticks and
 * 5100 right ticks, giving the initial right gain of 0.902. Floor runs then
 * bracketed straight travel: gain 902 gave 3859/4315 ticks and steered left;
 * gain 805 gave 4044/3797 ticks and steered right. After normalization by
 * 827.2 left and 825.2 right ticks/rev, their L/R ratios are 0.89216 and
 * 1.06248. Linear interpolation to L/R=1 gives gain 840.58, rounded to 841.
 * Lateral-error interpolation independently gives 842.81.
 * Derating the faster wheel, rather than boosting the slower one, preserves
 * the command path's configured duty ceilings.
 * Fixed-point per-mille avoids floating-point work in the control loop.
 * This is unloaded open-loop matching, not a substitute for wheel-speed PID.
 */
#define MOTOR_LEFT_GAIN_PERMILLE   1000
#define MOTOR_RIGHT_GAIN_PERMILLE   841


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
#define TRACK_WIDTH_M     0.2009734f /* wheel-to-wheel; calibrate on the rover */

/* ---- Relative turn procedure ---- */
#define TURN_RATE_RAD_S          DEG2RAD(50) /* Degrees per second turn rate */
#define TURN_OVERSHOOT_RAD       DEG2RAD(1)  /* stop that degree early */
#define TURN_LINEAR_LIMIT_M_S    0.15f

/* ---- Control loop ---- */
#define CONTROL_HZ        100   /* reflexive control-loop rate */

/* ---- Tunable defaults ---- */
/* Floor-measured 2026-08-12: 3908 ticks over 1020 mm,
     seeded from 826 ticks/rev and a 68 mm wheel. */
#define DEFAULT_TICKS_PER_METER  3831.0f

#define DEFAULT_PID_KP           0.5f
#define DEFAULT_PID_KI           0.1f
#define DEFAULT_PID_KD           0.0f
#define DEFAULT_MAX_PWM          1000      /* per-mille clamp (0..1000) */
#define DEFAULT_MAX_SPEED_MM_S   600       /* open-loop full-scale; calibrate */
#define DEFAULT_WATCHDOG_10MS    50        /* 50 * 10 ms = 500 ms */
#define DEFAULT_OBSTACLE_STOP_MM 0         /* disabled until ToF is wired */

/* ---- Firmware version (reported in INFO registers) ---- */
#define FW_VERSION_MAJOR  0
#define FW_VERSION_MINOR  11

#endif /* WANDERER_CONFIG_H */
