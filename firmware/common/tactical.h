#pragma once
#include <stdint.h>
#include "protocol.h"      // rf_protocol::TacticalState

using rf_protocol::TacticalState;

// Silence that separates a normal link blip from real loss.
constexpr uint32_t LIVENESS_TIMEOUT_MS  = 750;
constexpr int32_t  FALLBACK_DECEL_MM_S2 = 800;   // bounded decel in (mm/s)/s

// Latched fault codes. Grows with Tier 3 (overcurrent, tilt, ...).
#define FAULT_NONE   0
#define FAULT_ESTOP  1

// Command retcodes. 0 = success; negative = refused, one code per reason in
// the cockpit spec's error registry (protocol/cockpit_protocol.md, section 6/7).
#define TAC_OK                  0
#define TAC_ERR_NOT_ARMED      -1
#define TAC_ERR_FAULT_LATCHED  -2
#define TAC_ERR_NO_FAULT       -3
#define TAC_ERR_FAULT_PERSISTS -4
#define TAC_ERR_INVALID_FAULT  -5

// The Wanderer's vehicle FSM. One vehicle, one FSM: this is a module with
// internal state, not a class -- there is never a second instance.
//
// Commands are plain calls returning a retcode. tac_tick() dispatches the
// current state's time-driven behavior (deadman and fallback ramp).
//
// No hardware or OS dependencies: time comes in as a caller-supplied
// microsecond timestamp (`to_us_since_boot(get_absolute_time())` on the Pico,
// anything monotonic in host tests), so the module unit-tests on the host.

// Reset to power-on state (SAFE, everything zero). Call once at boot;
// tests call it between cases. Keeps registered callbacks.
void tac_init(void);

// Observation, for the protocol layer to emit `!state` / `!fault` lines.
// The fault code is latched before the callback fires, so on a transition to
// Fault the handler reads it via tac_fault_code(). May be NULL.
typedef void (*tac_state_cb)(TacticalState from, TacticalState to);
void tac_set_change_state_callback(tac_state_cb on_change_state);

// Commands (cockpit spec section 6, one function per row).
int tac_arm(void);
int tac_disarm(void);
int tac_stop(void);                              // zero velocity, REMAIN Active
int tac_drive(int16_t left_mm_s, int16_t right_mm_s);
int tac_estop(void);                             // = tac_raise_fault(FAULT_ESTOP)
int tac_raise_fault(uint16_t code);              // rejects FAULT_NONE; first cause wins
int tac_clear_fault(bool condition_cleared);

// Retcode -> wire reason token, for the cockpit `=err` line.
const char *tac_strerror(int rc);

// Wire names for states and fault codes (cockpit spec sections 3, 8).
const char *tac_state_name(TacticalState s);
const char *tac_fault_name(uint16_t code);

// Liveness: ANY valid commander frame, fed at the transport boundary.
void tac_note_commander_alive(uint64_t now_us);
bool is_tac_commander_alive(uint64_t now_us);

// Periodic: run the current state function. Call every loop.
void tac_tick(uint64_t now_us);

// Outputs; the motor layer consumes these.
bool          tac_motors_enabled(void);
int16_t       tac_target_left(void);
int16_t       tac_target_right(void);
TacticalState tac_state(void);
uint16_t      tac_fault_code(void);
