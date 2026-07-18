#include "tactical.h"

// ---- module state ---------------------------------------------------------

static void SafeState(void);
static void ActiveState(void);
static void FallbackState(void);
static void FaultState(void);

static TacticalState s_state    = TacticalState::Safe;

static int16_t  s_cmdLeft, s_cmdRight;      // standing order
static int16_t  s_rampLeft, s_rampRight;    // fallback ramp output
static uint16_t s_faultCode = FAULT_NONE;

static bool     s_seen;                     // any commander frame ever
static uint64_t s_lastSeenUs;
static uint64_t s_nowUs;
static uint64_t s_lastTickUs;

static tac_state_cb s_onChangeState;

void tac_init(void)
{
    s_state = TacticalState::Safe;
    s_cmdLeft = s_cmdRight = s_rampLeft = s_rampRight = 0;
    s_faultCode = FAULT_NONE;
    s_seen = false;
    s_lastSeenUs = s_nowUs = s_lastTickUs = 0;
}

void tac_set_change_state_callback(tac_state_cb on_change_state)
{
    s_onChangeState = on_change_state;
}

// ---- state transition and notification ------------------------------------

static void transition_to(TacticalState next)
{
    const TacticalState previous = s_state;
    if (previous == next)
        return;
    s_state = next;
    if (s_onChangeState)
        s_onChangeState(previous, next);
}

static void clear_targets(void)
{
    s_cmdLeft = s_cmdRight = s_rampLeft = s_rampRight = 0;
}


// ---- commands (cockpit spec section 6, one function per row) --------------

int tac_arm(void)
{
    if (s_state == TacticalState::Fault)
        return TAC_ERR_FAULT_LATCHED;
    if (s_state == TacticalState::Safe)
        transition_to(TacticalState::Active);
    return TAC_OK;       // ACTIVE/FALLBACK: no-op ok; arm never exits FALLBACK
}

int tac_disarm(void)
{
    if (s_state == TacticalState::Fault)
        return TAC_ERR_FAULT_LATCHED;
    clear_targets();
    transition_to(TacticalState::Safe);
    return TAC_OK;
}

// Zero the commanded velocity, REMAIN Active. (The RF-era CMD_STOP that
// disarmed is tac_disarm now.)
int tac_stop(void)
{
    if (s_state != TacticalState::Active)
        return TAC_ERR_NOT_ARMED;
    s_cmdLeft = s_cmdRight = 0;
    return TAC_OK;
}

int tac_drive(int16_t left_mm_s, int16_t right_mm_s)
{
    if (s_state != TacticalState::Active && s_state != TacticalState::Fallback)
        return TAC_ERR_NOT_ARMED;
    s_cmdLeft = left_mm_s;
    s_cmdRight = right_mm_s;
    if (s_state == TacticalState::Fallback)      // a fresh drive is the ONLY
        transition_to(TacticalState::Active);    // way out of FALLBACK upward
    return TAC_OK;
}

int tac_estop(void)
{
    return tac_raise_fault(FAULT_ESTOP);
}

int tac_raise_fault(uint16_t code)
{
    if (code == FAULT_NONE)
        return TAC_ERR_INVALID_FAULT;
    if (s_state == TacticalState::Fault)
        return TAC_OK;               // preserve the first latched cause
    s_faultCode = code;              // latch before the callback reads it
    clear_targets();
    transition_to(TacticalState::Fault);
    return TAC_OK;
}

int tac_clear_fault(bool condition_cleared)
{
    if (s_state != TacticalState::Fault)
        return TAC_ERR_NO_FAULT;
    if (!condition_cleared)
        return TAC_ERR_FAULT_PERSISTS;
    s_faultCode = FAULT_NONE;
    clear_targets();
    transition_to(TacticalState::Safe);
    return TAC_OK;
}

const char *tac_strerror(int rc)
{
    switch (rc) {
    case TAC_OK:                 return "ok";
    case TAC_ERR_NOT_ARMED:      return "not_armed";
    case TAC_ERR_FAULT_LATCHED:  return "fault_latched";
    case TAC_ERR_NO_FAULT:       return "no_fault";
    case TAC_ERR_FAULT_PERSISTS: return "fault_persists";
    case TAC_ERR_INVALID_FAULT:  return "invalid_fault";
    }
    return "unknown";
}

const char *tac_state_name(TacticalState s)
{
    switch (s) {
    case TacticalState::Safe:     return "SAFE";
    case TacticalState::Active:   return "ACTIVE";
    case TacticalState::Fallback: return "FALLBACK";
    case TacticalState::Fault:    return "FAULT";
    }
    return "?";
}

const char *tac_fault_name(uint16_t code)
{
    switch (code) {
    case FAULT_NONE:  return "NONE";
    case FAULT_ESTOP: return "ESTOP";
    }
    return "?";
}

// ---- liveness -------------------------------------------------------------

void tac_note_commander_alive(uint64_t now_us)
{
    s_seen = true;
    s_lastSeenUs = now_us;
}

bool is_tac_commander_alive(uint64_t now_us)
{
    return s_seen && (now_us - s_lastSeenUs) < LIVENESS_TIMEOUT_MS * 1000ull;
}

// ---- state functions: only the time-driven behavior lives here ------------

static void SafeState(void)
{
}

static int16_t decay(int16_t v, uint64_t elapsed_us)
{
    const uint64_t step = (uint64_t)FALLBACK_DECEL_MM_S2 * elapsed_us / 1000000;
    if (v > 0)
        return step >= (uint16_t)v ? 0 : (int16_t)((int32_t)v - (int32_t)step);
    if (v < 0) {
        const uint32_t magnitude = (uint32_t)(-(int32_t)v);
        return step >= magnitude ? 0 : (int16_t)((int32_t)v + (int32_t)step);
    }
    return 0;
}

static void apply_fallback_ramp(uint64_t elapsed_us)
{
    s_rampLeft  = decay(s_rampLeft, elapsed_us);
    s_rampRight = decay(s_rampRight, elapsed_us);
}

static void ActiveState(void)    // the deadman
{
    if (is_tac_commander_alive(s_nowUs))
        return;

    s_rampLeft = s_cmdLeft;
    s_rampRight = s_cmdRight;
    transition_to(TacticalState::Fallback);
    if (s_seen) {
        const uint64_t timeout_us = (uint64_t)LIVENESS_TIMEOUT_MS * 1000;
        const uint64_t elapsed_us = s_nowUs - s_lastSeenUs;
        if (elapsed_us > timeout_us)
            apply_fallback_ramp(elapsed_us - timeout_us);
    }
}

static void FallbackState(void)  // bounded decel ramp toward zero; stays here
{                                // until drive (resume), disarm, or a fault
    apply_fallback_ramp(s_nowUs - s_lastTickUs);
}

static void FaultState(void)
{
}

// ---- periodic -------------------------------------------------------------

void tac_tick(uint64_t now_us)
{
    s_nowUs = now_us;
    switch (s_state) {
    case TacticalState::Safe:     SafeState();     break;
    case TacticalState::Active:   ActiveState();   break;
    case TacticalState::Fallback: FallbackState(); break;
    case TacticalState::Fault:    FaultState();    break;
    }
    s_lastTickUs = now_us;
}

// ---- outputs --------------------------------------------------------------

bool tac_motors_enabled(void)
{
    return s_state == TacticalState::Active || s_state == TacticalState::Fallback;
}

int16_t tac_target_left(void)
{
    if (s_state == TacticalState::Active)   return s_cmdLeft;
    if (s_state == TacticalState::Fallback) return s_rampLeft;
    return 0;
}

int16_t tac_target_right(void)
{
    if (s_state == TacticalState::Active)   return s_cmdRight;
    if (s_state == TacticalState::Fallback) return s_rampRight;
    return 0;
}

TacticalState tac_state(void) { return s_state; }
uint16_t tac_fault_code(void) { return s_faultCode; }
