#include "tactical.h"

// ---- module state ---------------------------------------------------------

typedef void (*FuncPtr)(void);

static void SafeState(void);
static void ActiveState(void);
static void FallbackState(void);
static void FaultState(void);

static FuncPtr       s_nextFunc = SafeState;
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
    s_nextFunc = SafeState;
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

// ---- transition: entry actions + notification, immediate ------------------

static void go(TacticalState s)
{
    if (s == s_state)
        return;
    switch (s) {
    case TacticalState::Safe:     s_nextFunc = SafeState;     break;
    case TacticalState::Active:   s_nextFunc = ActiveState;   break;
    case TacticalState::Fallback: s_nextFunc = FallbackState; break;
    case TacticalState::Fault:    s_nextFunc = FaultState;    break;
    }
    if (s == TacticalState::Fallback) {          // seed ramp from standing order
        s_rampLeft  = s_cmdLeft;
        s_rampRight = s_cmdRight;
    }
    if (s == TacticalState::Safe || s == TacticalState::Fault)
        s_cmdLeft = s_cmdRight = s_rampLeft = s_rampRight = 0;

    TacticalState from = s_state;
    s_state = s;
    if (s_onChangeState)
        s_onChangeState(from, s);
}

// ---- commands (cockpit spec section 6, one function per row) --------------

int tac_arm(void)
{
    if (s_state == TacticalState::Fault)
        return TAC_ERR_FAULT_LATCHED;
    if (s_state == TacticalState::Safe)
        go(TacticalState::Active);
    return TAC_OK;       // ACTIVE/FALLBACK: no-op ok; arm never exits FALLBACK
}

int tac_disarm(void)
{
    if (s_state == TacticalState::Fault)
        return TAC_ERR_FAULT_LATCHED;
    go(TacticalState::Safe);
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
        go(TacticalState::Active);               // way out of FALLBACK upward
    return TAC_OK;
}

int tac_estop(void)
{
    return tac_raise_fault(FAULT_ESTOP);
}

int tac_raise_fault(uint16_t code)
{
    s_faultCode = code;      // latched BEFORE go(), so the state callback
    if (s_state != TacticalState::Fault)         // reads it via tac_fault_code()
        go(TacticalState::Fault);
    return TAC_OK;
}

int tac_clear_fault(bool condition_cleared)
{
    if (s_state != TacticalState::Fault)
        return TAC_ERR_NO_FAULT;
    if (!condition_cleared)
        return TAC_ERR_FAULT_PERSISTS;
    s_faultCode = FAULT_NONE;
    go(TacticalState::Safe);
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
    }
    return "unknown";
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

static void ActiveState(void)    // the deadman
{
    if (!is_tac_commander_alive(s_nowUs))
        go(TacticalState::Fallback);
}

static int16_t decay(int16_t v, int32_t step)
{
    if (v > 0) return (int16_t)(v >  step ? v - step : 0);
    if (v < 0) return (int16_t)(-v > step ? v + step : 0);
    return 0;
}

static void FallbackState(void)  // bounded decel ramp toward zero; stays here
{                                // until drive (resume), disarm, or a fault
    int32_t step = (int32_t)((int64_t)FALLBACK_DECEL_MM_S2
                             * (int64_t)(s_nowUs - s_lastTickUs) / 1000000);
    s_rampLeft  = decay(s_rampLeft, step);
    s_rampRight = decay(s_rampRight, step);
}

static void FaultState(void)
{
}

// ---- periodic -------------------------------------------------------------

void tac_tick(uint64_t now_us)
{
    s_nowUs = now_us;
    s_nextFunc();
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
