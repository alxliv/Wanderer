#include "i2c_peripheral.h"
#include "i2c_registers.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/i2c_slave.h"
#include <string.h>

/* Backing register image and transaction state. Accessed from the I2C IRQ
 * (handler) and the main loop (typed accessors). */
static struct {
    i2c_inst_t *i2c;
    uint8_t mem[WANDERER_REG_SPACE_SIZE];   /* canonical register image */
    uint8_t rdbuf[WANDERER_REG_SPACE_SIZE]; /* snapshot served during a read */
    uint8_t wrbuf[WANDERER_REG_SPACE_SIZE]; /* staged write transaction */
    bool    dirty[WANDERER_REG_SPACE_SIZE];
    uint8_t ptr;          /* current register pointer */
    bool    ptr_set;      /* first byte of a write sets the pointer */
    bool    reading;      /* a read snapshot has been taken this transaction */
    bool    write_dirty;  /* writable bytes staged in this transaction */
    volatile bool cmd_written; /* master wrote CONTROL/COMMAND region */
    volatile bool cfg_written; /* master wrote CONFIG region */
} s;

/* Only CONTROL/COMMAND (0x10-0x1F) and CONFIG (0x40-0x5F) are writable by the
 * master; INFO and TELEMETRY are read-only and ignore writes. */
static inline bool reg_writable(uint8_t a) {
    return (a >= 0x10u && a <= 0x1Fu) || (a >= 0x40u && a <= 0x5Fu);
}

static void commit_staged_write(void) {
    if (!s.write_dirty) {
        return;
    }

    bool cmd_written = false;
    bool cfg_written = false;

    for (uint8_t i = 0; i < WANDERER_REG_SPACE_SIZE; i++) {
        if (!s.dirty[i]) {
            continue;
        }

        s.mem[i] = s.wrbuf[i];
        if (i >= 0x10u && i <= 0x1Fu) {
            cmd_written = true;
        } else if (i >= 0x40u && i <= 0x5Fu) {
            cfg_written = true;
        }
        s.dirty[i] = false;
    }

    if (cmd_written) {
        s.cmd_written = true;
    }
    if (cfg_written) {
        s.cfg_written = true;
    }
    s.write_dirty = false;
}

/* I2C slave event handler (runs in IRQ context). Implements the
 * register-pointer model with command-apply-on-STOP and read snapshotting. */
static void on_i2c_event(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: {
        uint8_t b = i2c_read_byte_raw(i2c);
        if (!s.ptr_set) {
            s.ptr = b;          /* first byte = register address */
            s.ptr_set = true;
        } else {
            if (reg_writable(s.ptr)) {
                s.wrbuf[s.ptr] = b;
                s.dirty[s.ptr] = true;
                s.write_dirty = true;
            }
            s.ptr++;
        }
        break;
    }
    case I2C_SLAVE_REQUEST:
        if (!s.reading) {       /* snapshot at the start of a read = no tearing */
            memcpy(s.rdbuf, s.mem, sizeof s.rdbuf);
            s.reading = true;
        }
        i2c_write_byte_raw(i2c,
            (s.ptr < WANDERER_REG_SPACE_SIZE) ? s.rdbuf[s.ptr] : 0u);
        s.ptr++;
        break;

    case I2C_SLAVE_FINISH:      /* STOP or repeated START with new address */
        commit_staged_write();
        s.ptr_set = false;
        s.reading = false;
        break;

    default:
        break;
    }
}

void i2cp_init(i2c_inst_t *i2c, uint sda_pin, uint scl_pin, uint baud, uint8_t addr) {
    s.i2c = i2c;
    memset(s.mem, 0, sizeof s.mem);
    memset(s.rdbuf, 0, sizeof s.rdbuf);
    memset(s.wrbuf, 0, sizeof s.wrbuf);
    memset(s.dirty, 0, sizeof s.dirty);
    s.ptr = 0;
    s.ptr_set = false;
    s.reading = false;
    s.write_dirty = false;
    s.cmd_written = false;
    s.cfg_written = false;

    i2c_init(i2c, baud);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    /* Internal pull-ups as a fallback; external ~4.7k pull-ups recommended on
     * the bus to the host (see hardware/wiring.md). */
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    i2c_slave_init(i2c, addr, &on_i2c_event);
}

/* ---- typed accessors ---- */

uint8_t i2cp_get_u8(uint8_t reg) { return s.mem[reg]; }
void    i2cp_set_u8(uint8_t reg, uint8_t v) { s.mem[reg] = v; }

#define GUARDED_GET(TYPE, REG)                              \
    TYPE v;                                                 \
    uint32_t fl = save_and_disable_interrupts();            \
    memcpy(&v, &s.mem[(REG)], sizeof(TYPE));                \
    restore_interrupts(fl);                                 \
    return v

#define GUARDED_SET(TYPE, REG, VAL)                         \
    TYPE tmp = (VAL);                                       \
    uint32_t fl = save_and_disable_interrupts();            \
    memcpy(&s.mem[(REG)], &tmp, sizeof(TYPE));              \
    restore_interrupts(fl)

uint16_t i2cp_get_u16(uint8_t reg)            { GUARDED_GET(uint16_t, reg); }
void     i2cp_set_u16(uint8_t reg, uint16_t v){ GUARDED_SET(uint16_t, reg, v); }
int16_t  i2cp_get_i16(uint8_t reg)            { GUARDED_GET(int16_t, reg); }
void     i2cp_set_i16(uint8_t reg, int16_t v) { GUARDED_SET(int16_t, reg, v); }
int32_t  i2cp_get_i32(uint8_t reg)            { GUARDED_GET(int32_t, reg); }
void     i2cp_set_i32(uint8_t reg, int32_t v) { GUARDED_SET(int32_t, reg, v); }
float    i2cp_get_f32(uint8_t reg)            { GUARDED_GET(float, reg); }
void     i2cp_set_f32(uint8_t reg, float v)   { GUARDED_SET(float, reg, v); }

bool i2cp_take_command_written(void) {
    uint32_t fl = save_and_disable_interrupts();
    bool b = s.cmd_written;
    s.cmd_written = false;
    restore_interrupts(fl);
    return b;
}

bool i2cp_take_config_written(void) {
    uint32_t fl = save_and_disable_interrupts();
    bool b = s.cfg_written;
    s.cfg_written = false;
    restore_interrupts(fl);
    return b;
}
