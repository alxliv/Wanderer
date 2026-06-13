/*
 * Wanderer - tactical-host <-> Pico 2 I2C register map.
 *
 * Single source of truth for the reflexive-tier interface. Mirrors
 * protocol/i2c_registers.md exactly. Included directly by the Pico firmware;
 * the host's Python side (Raspberry Pi Zero 2 W) mirrors these constants.
 *
 * Pico = I2C peripheral @ 0x42. All multi-byte fields are LITTLE-ENDIAN.
 * Floats are IEEE-754 float32, little-endian (native to RP2350 and the host).
 */
#ifndef WANDERER_I2C_REGISTERS_H
#define WANDERER_I2C_REGISTERS_H

#include <stdint.h>

#define WANDERER_PROTOCOL_VERSION 1u
#define WANDERER_I2C_ADDR         0x42u
#define WANDERER_DEVICE_ID        0x57u /* 'W' */

/* ---- INFO (read-only) 0x00-0x0F ---- */
#define REG_PROTOCOL_VERSION 0x00u /* u8 */
#define REG_FW_VERSION_MAJOR 0x01u /* u8 */
#define REG_FW_VERSION_MINOR 0x02u /* u8 */
#define REG_DEVICE_ID        0x03u /* u8 = 0x57 */

/* ---- CONTROL / COMMAND (read/write) 0x10-0x1F ---- */
#define REG_CONTROL_MODE     0x10u /* u8  */
#define REG_CONTROL_FLAGS    0x11u /* u8  */
#define REG_CMD_LEFT         0x12u /* i16 */
#define REG_CMD_RIGHT        0x14u /* i16 */
#define REG_WATCHDOG_TIMEOUT 0x16u /* u8, x10ms; 0 = disabled */

/* CONTROL_MODE values */
#define MODE_IDLE       0u
#define MODE_VELOCITY   1u
#define MODE_DIRECT_PWM 2u

/* CONTROL_FLAGS bits */
#define FLAG_MOTOR_ENABLE  (1u << 0) /* level  */
#define FLAG_CLEAR_FAULTS  (1u << 1) /* action, self-clearing */
#define FLAG_RESET_ODOM    (1u << 2) /* action, self-clearing */

/* ---- TELEMETRY (read-only) 0x20-0x3F ---- */
#define REG_STATUS        0x20u /* u8  */
#define REG_FAULT         0x21u /* u8  */
#define REG_MEAS_LEFT     0x22u /* i16, mm/s */
#define REG_MEAS_RIGHT    0x24u /* i16, mm/s */
#define REG_ENC_LEFT      0x26u /* i32, ticks */
#define REG_ENC_RIGHT     0x2Au /* i32, ticks */
#define REG_TOF_FRONT_MM  0x2Eu /* u16, mm (0xFFFF = invalid) */
#define REG_TOF_STATUS    0x30u /* u8  */
#define REG_LOOP_HZ       0x32u /* u16 */
#define REG_VBAT_MOTOR_MV 0x34u /* u16, reserved/future */
#define REG_VBAT_LOGIC_MV 0x36u /* u16, reserved/future */

/* STATUS bits */
#define ST_MOTORS_ENABLED  (1u << 0)
#define ST_WATCHDOG_TRIPPED (1u << 1)
#define ST_OBSTACLE_STOP   (1u << 2)
#define ST_LEFT_AT_TARGET  (1u << 3)
#define ST_RIGHT_AT_TARGET (1u << 4)
#define ST_ANY_FAULT       (1u << 5)
#define ST_MODE_MASK       (3u << 6) /* bits 6-7 = current mode */
#define ST_MODE_SHIFT      6u

/* FAULT bits */
#define FT_STALL_LEFT      (1u << 0)
#define FT_STALL_RIGHT     (1u << 1)
#define FT_TOF_ERROR       (1u << 2)
#define FT_ENC_LEFT_ERROR  (1u << 3)
#define FT_ENC_RIGHT_ERROR (1u << 4)
#define FT_OVERCURRENT     (1u << 5) /* reserved/future */
#define FT_LOW_VOLTAGE     (1u << 6) /* reserved/future */
#define FT_WATCHDOG        (1u << 7)

/* ---- CONFIG (read/write) 0x40-0x5F ---- */
#define REG_TICKS_PER_METER 0x40u /* f32, x4 quadrature edges per meter */
#define REG_PID_KP          0x44u /* f32 */
#define REG_PID_KI          0x48u /* f32 */
#define REG_PID_KD          0x4Cu /* f32 */
#define REG_MAX_PWM         0x50u /* u16, 0..1000 */
#define REG_OBSTACLE_STOP_MM 0x54u /* u16, mm; 0 = disabled */

#define WANDERER_REG_SPACE_SIZE 0x60u /* total addressable register bytes */

/*
 * Optional struct overlay of the register space (little-endian targets only).
 * The firmware may back the I2C peripheral with this layout. Offsets must match
 * the addresses above exactly; reserved arrays pad the gaps.
 */
#pragma pack(push, 1)
typedef struct {
    /* INFO 0x00 */
    uint8_t  protocol_version;     /* 0x00 */
    uint8_t  fw_version_major;     /* 0x01 */
    uint8_t  fw_version_minor;     /* 0x02 */
    uint8_t  device_id;            /* 0x03 */
    uint8_t  _rsv_info[12];        /* 0x04-0x0F */
    /* CONTROL / COMMAND 0x10 */
    uint8_t  control_mode;         /* 0x10 */
    uint8_t  control_flags;        /* 0x11 */
    int16_t  cmd_left;             /* 0x12 */
    int16_t  cmd_right;            /* 0x14 */
    uint8_t  watchdog_timeout;     /* 0x16 */
    uint8_t  _rsv_cmd[9];          /* 0x17-0x1F */
    /* TELEMETRY 0x20 */
    uint8_t  status;               /* 0x20 */
    uint8_t  fault;                /* 0x21 */
    int16_t  meas_left;            /* 0x22 */
    int16_t  meas_right;           /* 0x24 */
    int32_t  enc_left;             /* 0x26 */
    int32_t  enc_right;            /* 0x2A */
    uint16_t tof_front_mm;         /* 0x2E */
    uint8_t  tof_status;           /* 0x30 */
    uint8_t  _rsv_tel0;            /* 0x31 */
    uint16_t loop_hz;              /* 0x32 */
    uint16_t vbat_motor_mv;        /* 0x34 */
    uint16_t vbat_logic_mv;        /* 0x36 */
    uint8_t  _rsv_tel1[8];         /* 0x38-0x3F */
    /* CONFIG 0x40 */
    float    ticks_per_meter;      /* 0x40 */
    float    pid_kp;               /* 0x44 */
    float    pid_ki;               /* 0x48 */
    float    pid_kd;               /* 0x4C */
    uint16_t max_pwm;              /* 0x50 */
    uint16_t _rsv_cfg0;            /* 0x52 */
    uint16_t obstacle_stop_mm;     /* 0x54 */
    uint8_t  _rsv_cfg1[10];        /* 0x56-0x5F */
} wanderer_regs_t;
#pragma pack(pop)

#endif /* WANDERER_I2C_REGISTERS_H */
