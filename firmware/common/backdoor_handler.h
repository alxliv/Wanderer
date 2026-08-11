#pragma once
#include <stdint.h>
#include <stdbool.h>

// The airframe's SYSTEM BACKDOOR endpoint -- the maintenance hatch of
// architecture section 3a, carried on USB CDC (the bench transport) rather
// than the cockpit UART.
//
// Membership is CLOSED. Every verb here belongs to one of exactly the three
// categories the architecture admits, and the category is named in the table
// below so that adding a fourth kind of verb requires editing this comment
// and noticing what you are doing:
//
//   verb                        category
//   --------------------------  ---------------------------------------------
//   estop                       emergency override
//   safe                        emergency override
//   clear_fault                 emergency override (undoes estop)
//   dev on | dev off            (the gate itself -- authority, not operation)
//   wiggle <l> <r> <ms>         dev-time diagnostic (raw bench wiggle)
//   enc [reset]                 dev-time diagnostic (read a counter)
//   ver                         dev-time diagnostic (identify)
//   help                        dev-time diagnostic (list the above)
//
// NOTHING may be added merely because routing here was convenient. Normal
// motion flies through the cockpit (Pi5 / UART), full stop. In particular
// there is deliberately NO streaming motion verb: `wiggle` is one-shot and
// self-terminating, so a dropped USB cable stops the wheels instead of
// latching them on.
//
// `clear_fault` earns its place because `estop` does: a bench session can
// be running with no cockpit attached at all, so a verb capable of latching
// FAULT must be matched by one capable of lifting it, or the operator is
// stuck until they go find a Pi5. Same semantics as the cockpit's own
// `clear_fault` (tac_clear_fault(true) -- Tier 3 faults will plug a real
// condition check in here on both interfaces at once).
//
// Transport-agnostic in the same way cockpit_handler is: feed it bytes, give
// it sinks. It owns its OWN LineAssembler, so the airframe runs this and the
// cockpit concurrently on two ports without either seeing the other's bytes.
//
// Deliberately does NOT register a tac_set_change_state_callback: tactical
// holds one callback and it belongs to the cockpit. The bench operator learns
// what happened from the reply line to the request they just sent.

// Bench safety rails. Both are hard caps applied to every wiggle, not
// defaults -- the operator cannot raise them from the wire.
#define BACKDOOR_MAX_DUTY_PERMILLE  600    // 60% duty; enough to break stiction
#define BACKDOOR_MAX_WIGGLE_MS      3000   // no wiggle outlives 3 seconds

// One outgoing line, NUL-terminated, WITHOUT terminator; the transport
// appends "\r\n".
typedef void (*backdoor_sink)(const char *line);

// Raw per-mille motion, straight to the motor driver, bypassing the mm/s
// mapping (whose full-scale constant is exactly what calibration is trying to
// measure -- routing bench motion through it would be circular). The airframe
// applies these ONLY while tac_dev_active().
typedef void (*backdoor_motor_fn)(int16_t left_permille, int16_t right_permille);

// Raw cumulative encoder counts; NULL reports zeros.
typedef void (*backdoor_enc_fn)(int32_t *left_ticks, int32_t *right_ticks);

// Zero the encoder counts (the `enc reset` verb). May be NULL.
typedef void (*backdoor_enc_reset_fn)(void);

void backdoor_init(backdoor_sink sink, uint8_t fw_major, uint8_t fw_minor);
void backdoor_set_motor_sink(backdoor_motor_fn fn);
void backdoor_set_encoder_provider(backdoor_enc_fn fn, backdoor_enc_reset_fn reset);

// Pump one received byte; dispatches when a full line has arrived.
void backdoor_feed(char c, uint64_t now_us);

// Call every control period. Enforces the wiggle deadline and stops the
// wheels the instant the dev lease is lost (revoked by cockpit traffic, a
// fault, or ESTOP from any source).
void backdoor_tick(uint64_t now_us);

// The transport went away (USB disconnected). Stops any wiggle, drops the
// lease, and resets line assembly so a reconnecting terminal starts clean.
void backdoor_abort(void);

// True while a wiggle is outstanding -- the airframe's motor mux uses this
// together with tac_dev_active().
bool backdoor_wiggle_active(void);
