// Host-side tests for the tactical FSM against the cockpit spec section 6
// matrix.
// Build:  g++ -std=c++17 -Wall -Wextra -I. test_tactical.cpp tactical.cpp -o test_tactical
// Run:    ./test_tactical        (exit code 0 = all pass)
//
// Part 1 walks every command cell of the matrix (rows = commands, columns =
// states). The "queries" row never reaches the FSM -- it is wire-level,
// tested in the Python suite. Part 2 covers what the matrix cannot express:
// deadman timing, the fallback ramp, callback order.

#include <stdint.h>
#include <stdio.h>
#include "tactical.h"

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, msg); failures++; } \
} while (0)

static const char *name(TacticalState s)
{
    switch (s) {
    case TacticalState::Safe:     return "SAFE";
    case TacticalState::Active:   return "ACTIVE";
    case TacticalState::Fallback: return "FALLBACK";
    case TacticalState::Fault:    return "FAULT";
    }
    return "?";
}

// Records callback firings so tests can assert events and their absence.
// On entry to Fault it snapshots tac_fault_code(), proving the code is
// latched before the callback fires.
static int rec_count;
static int rec_to[8];
static int rec_code[8];

static void rec_clear(void) { rec_count = 0; }
static void rec_on_state(TacticalState, TacticalState to)
{
    if (rec_count < 8) {
        rec_to[rec_count] = (int)to;
        rec_code[rec_count] = (to == TacticalState::Fault) ? tac_fault_code() : -1;
        rec_count++;
    }
}

// Reset the FSM and drive it into the wanted state. Time is virtual; caller
// continues from the returned timestamp.
static uint64_t enter(TacticalState target)
{
    uint64_t now = 1000000;    // t = 1 s
    tac_init();
    if (target == TacticalState::Safe)
        return now;
    tac_note_commander_alive(now);
    tac_arm();
    if (target == TacticalState::Active)
        return now;
    if (target == TacticalState::Fallback) {
        tac_drive(300, 300);
        now += (LIVENESS_TIMEOUT_MS + 50) * 1000ull;
        tac_tick(now);         // lease lapsed -> FALLBACK
        return now;
    }
    tac_estop();               // -> FAULT
    return now;
}

// ---- Part 1: the matrix ---------------------------------------------------

enum Cmd { ARM, DISARM, STOP, DRIVE, ESTOP, CLEAR_OK, CLEAR_BLOCKED };

struct Cell {
    Cmd cmd;
    TacticalState in_state;
    TacticalState expect_state;
    int expect_rc;
};

// Transcription of cockpit spec section 6. clear_fault appears twice in the
// FAULT column: condition cleared (-> SAFE) and persisting (latch kept).
static const Cell MATRIX[] = {
    // arm
    { ARM,     TacticalState::Safe,     TacticalState::Active,   TAC_OK },
    { ARM,     TacticalState::Active,   TacticalState::Active,   TAC_OK },
    { ARM,     TacticalState::Fallback, TacticalState::Fallback, TAC_OK },
    { ARM,     TacticalState::Fault,    TacticalState::Fault,    TAC_ERR_FAULT_LATCHED },
    // disarm
    { DISARM,  TacticalState::Safe,     TacticalState::Safe,     TAC_OK },
    { DISARM,  TacticalState::Active,   TacticalState::Safe,     TAC_OK },
    { DISARM,  TacticalState::Fallback, TacticalState::Safe,     TAC_OK },
    { DISARM,  TacticalState::Fault,    TacticalState::Fault,    TAC_ERR_FAULT_LATCHED },
    // drive
    { DRIVE,   TacticalState::Safe,     TacticalState::Safe,     TAC_ERR_NOT_ARMED },
    { DRIVE,   TacticalState::Active,   TacticalState::Active,   TAC_OK },
    { DRIVE,   TacticalState::Fallback, TacticalState::Active,   TAC_OK },
    { DRIVE,   TacticalState::Fault,    TacticalState::Fault,    TAC_ERR_NOT_ARMED },
    // stop
    { STOP,    TacticalState::Safe,     TacticalState::Safe,     TAC_ERR_NOT_ARMED },
    { STOP,    TacticalState::Active,   TacticalState::Active,   TAC_OK },
    { STOP,    TacticalState::Fallback, TacticalState::Fallback, TAC_ERR_NOT_ARMED },
    { STOP,    TacticalState::Fault,    TacticalState::Fault,    TAC_ERR_NOT_ARMED },
    // estop
    { ESTOP,   TacticalState::Safe,     TacticalState::Fault,    TAC_OK },
    { ESTOP,   TacticalState::Active,   TacticalState::Fault,    TAC_OK },
    { ESTOP,   TacticalState::Fallback, TacticalState::Fault,    TAC_OK },
    { ESTOP,   TacticalState::Fault,    TacticalState::Fault,    TAC_OK },
    // clear_fault
    { CLEAR_OK,      TacticalState::Safe,     TacticalState::Safe,     TAC_ERR_NO_FAULT },
    { CLEAR_OK,      TacticalState::Active,   TacticalState::Active,   TAC_ERR_NO_FAULT },
    { CLEAR_OK,      TacticalState::Fallback, TacticalState::Fallback, TAC_ERR_NO_FAULT },
    { CLEAR_OK,      TacticalState::Fault,    TacticalState::Safe,     TAC_OK },
    { CLEAR_BLOCKED, TacticalState::Fault,    TacticalState::Fault,    TAC_ERR_FAULT_PERSISTS },
};

static void run_matrix(void)
{
    char msg[128];
    tac_set_change_state_callback(rec_on_state);

    for (const Cell &cell : MATRIX) {
        enter(cell.in_state);
        rec_clear();               // only the cell's own events from here

        int rc = 0;
        switch (cell.cmd) {
        case ARM:           rc = tac_arm(); break;
        case DISARM:        rc = tac_disarm(); break;
        case STOP:          rc = tac_stop(); break;
        case DRIVE:         rc = tac_drive(200, -200); break;
        case ESTOP:         rc = tac_estop(); break;
        case CLEAR_OK:      rc = tac_clear_fault(true); break;
        case CLEAR_BLOCKED: rc = tac_clear_fault(false); break;
        }

        snprintf(msg, sizeof msg, "cmd %d in %s: rc %d, expected %d",
                 cell.cmd, name(cell.in_state), rc, cell.expect_rc);
        CHECK(rc == cell.expect_rc, msg);
        snprintf(msg, sizeof msg, "cmd %d in %s: state %s, expected %s",
                 cell.cmd, name(cell.in_state),
                 name(tac_state()), name(cell.expect_state));
        CHECK(tac_state() == cell.expect_state, msg);

        // A refusal must change nothing and notify nothing.
        if (cell.expect_rc != TAC_OK) {
            snprintf(msg, sizeof msg, "cmd %d in %s: refusal fired a callback",
                     cell.cmd, name(cell.in_state));
            CHECK(rec_count == 0, msg);
        }
        // A transition must fire the state callback exactly once.
        if (cell.expect_state != cell.in_state) {
            snprintf(msg, sizeof msg, "cmd %d in %s: state callback count",
                     cell.cmd, name(cell.in_state));
            CHECK(rec_count == 1 && rec_to[0] == (int)cell.expect_state, msg);
        }
    }
}

// ---- Part 2: dynamics the matrix cannot express ---------------------------

static void run_dynamics(void)
{
    // Deadman: quiet commander -> FALLBACK; targets ramp, never step, to zero.
    {
        uint64_t now = enter(TacticalState::Active);
        tac_drive(400, -400);
        CHECK(tac_target_left() == 400 && tac_target_right() == -400, "drive applied");

        now += (LIVENESS_TIMEOUT_MS + 10) * 1000ull;
        tac_tick(now);
        CHECK(tac_state() == TacticalState::Fallback, "lease lapse -> FALLBACK");
        CHECK(tac_target_left() == 400, "ramp seeded from standing order");

        now += 100000;   // 100 ms at 800 mm/s^2 => 80 mm/s off
        tac_tick(now);
        CHECK(tac_target_left() == 320 && tac_target_right() == -320, "ramp step");
        CHECK(tac_motors_enabled(), "motors stay enabled through FALLBACK");

        for (int i = 0; i < 10; i++) { now += 100000; tac_tick(now); }
        CHECK(tac_target_left() == 0 && tac_target_right() == 0, "ramp reaches zero");
        CHECK(tac_state() == TacticalState::Fallback, "stays FALLBACK at zero");

        // arm must NOT resume; a fresh drive is the only way out upward.
        tac_note_commander_alive(now);
        tac_arm();
        CHECK(tac_state() == TacticalState::Fallback, "arm never exits FALLBACK");
        tac_drive(100, 100);
        CHECK(tac_state() == TacticalState::Active, "drive resumes");
        CHECK(tac_target_left() == 100, "resume applies new velocity");
    }
    // Entering SAFE gates outputs to zero immediately.
    {
        enter(TacticalState::Active);
        tac_drive(300, 300);
        tac_disarm();
        CHECK(tac_target_left() == 0 && !tac_motors_enabled(), "disarm gates to zero");
    }
    // On latch: one callback; fault code already latched when it fires.
    {
        enter(TacticalState::Active);
        rec_clear();
        tac_estop();
        CHECK(rec_count == 1, "estop notifies exactly once");
        CHECK(rec_count == 1 && rec_to[0] == (int)TacticalState::Fault,
              "callback entered Fault");
        CHECK(rec_count == 1 && rec_code[0] == FAULT_ESTOP,
              "code readable inside the callback");
        CHECK(tac_fault_code() == FAULT_ESTOP, "code latched");

        // Re-raise while latched: code updates, no notifications.
        rec_clear();
        tac_raise_fault(7);
        CHECK(rec_count == 0, "re-raise in FAULT notifies nothing");
        CHECK(tac_fault_code() == 7, "re-raise keeps latest code");

        tac_clear_fault(true);
        CHECK(tac_state() == TacticalState::Safe && tac_fault_code() == FAULT_NONE,
              "clear resets code");
    }
    // stop in ACTIVE: velocity zero, still armed.
    {
        enter(TacticalState::Active);
        tac_drive(250, 250);
        CHECK(tac_stop() == TAC_OK, "stop ok while ACTIVE");
        CHECK(tac_state() == TacticalState::Active, "stop stays ACTIVE");
        CHECK(tac_target_left() == 0 && tac_motors_enabled(), "zero velocity, armed");
    }
    // tac_strerror maps retcodes to wire reasons.
    {
        CHECK(tac_strerror(TAC_ERR_NOT_ARMED)[0] == 'n', "strerror not_armed");
    }
}

int main(void)
{
    run_matrix();
    run_dynamics();
    if (failures == 0)
        printf("OK: all matrix cells and dynamics pass\n");
    else
        printf("%d FAILURES\n", failures);
    return failures == 0 ? 0 : 1;
}
