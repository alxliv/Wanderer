/*
 * Wanderer reflexive layer - I2C peripheral (slave) interface to the Pi 5.
 *
 * Implements the register-pointer ("memory") access model described in
 * protocol/i2c_registers.md. Owns the backing register image; the main control
 * loop reads commands/config and writes telemetry through the typed accessors
 * below, which are safe against the I2C interrupt handler.
 */
#ifndef WANDERER_I2C_PERIPHERAL_H
#define WANDERER_I2C_PERIPHERAL_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

/* Initialize the I2C block as a peripheral at `addr`, with SDA/SCL pins and
 * bus speed. Installs the slave event handler. */
void i2cp_init(i2c_inst_t *i2c, uint sda_pin, uint scl_pin, uint baud, uint8_t addr);

/* Typed register access for the main loop.
 * Single-byte access is inherently atomic; multi-byte access is guarded
 * against the I2C IRQ so values never tear. All fields are little-endian,
 * matching both the protocol and the RP2350. */
uint8_t  i2cp_get_u8 (uint8_t reg);
void     i2cp_set_u8 (uint8_t reg, uint8_t v);
uint16_t i2cp_get_u16(uint8_t reg);
void     i2cp_set_u16(uint8_t reg, uint16_t v);
int16_t  i2cp_get_i16(uint8_t reg);
void     i2cp_set_i16(uint8_t reg, int16_t v);
int32_t  i2cp_get_i32(uint8_t reg);
void     i2cp_set_i32(uint8_t reg, int32_t v);
float    i2cp_get_f32(uint8_t reg);
void     i2cp_set_f32(uint8_t reg, float v);

/* Returns true once if the master wrote to the CONTROL/COMMAND region since the
 * previous call, then clears the flag. Used to refresh the watchdog. */
bool i2cp_take_command_written(void);

/* Returns true once if the master wrote to the CONFIG region since the previous
 * call, then clears the flag. Used to re-load tunables (PID, calibration). */
bool i2cp_take_config_written(void);

#endif /* WANDERER_I2C_PERIPHERAL_H */
