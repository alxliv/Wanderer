// Host-side tests for the system backdoor (architecture section 3a): the
// authority gate, the bench safety rails, and the interlocks that end a
// wiggle. Request lines go in through backdoor_feed(); reply and event lines
// come out through the sink. No hardware, no Pico SDK -- the motor "driver"
// is a pair of captured integers.
// Build: see CMakeLists.txt here, or:
//   g++ -std=c++17 -Wall -Wextra -I.. test_backdoor.cpp ../backdoor_handler.cpp
//       ../cockpit_codec.cpp ../tactical.cpp -o test_backdoor

#include <stdio.h>
#include <string.h>

#include "backdoor_handler.h"
#include "cockpit_codec.h"
#include "tactical.h"

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, msg); failures++; } \
} while (0)

// ---- captured output -------------------------------------------------------

#define MAX_LINES 16
static char out_lines[MAX_LINES][CODEC_MAX_LINE];
static int  out_count;

static void sink(const char *line)
{
    if (out_count < MAX_LINES)
        snprintf(out_lines[out_count], CODEC_MAX_LINE, "%s", line);
    out_count++;
}

static void out_clear(void) { out_count = 0; }

static const char *out_line(int i)
{
    return (i < out_count && i < MAX_LINES) ? out_lines[i] : "<missing>";
}

// Any captured line equal to `want`. The help text emits several `*` lines
// before its reply, so tests that only care about the reply search.
static bool out_has(const char *want)
{
    for (int i = 0; i < out_count && i < MAX_LINES; ++i)
        if (strcmp(out_lines[i], want) == 0)
            return true;
    return false;
}

// ---- fake motor driver -----------------------------------------------------

static int16_t mot_l, mot_r;
static int     mot_writes;

static void motor(int16_t l, int16_t r)
{
    mot_l = l;
    mot_r = r;
    mot_writes++;
}

// ---- fake encoders ---------------------------------------------------------

static int32_t enc_l = 1234, enc_r = -99;
static int     enc_resets;

static void encoders(int32_t *l, int32_t *r) { *l = enc_l; *r = enc_r; }
static void encoders_reset(void) { enc_l = enc_r = 0; enc_resets++; }

// ---- helpers ---------------------------------------------------------------

static uint64_t g_now = 1000000;   // virtual time, us

static void send(const char *line)
{
    for (const char *p = line; *p; ++p)
        backdoor_feed(*p, g_now);
    backdoor_feed('\n', g_now);
}

static void advance_ms(uint32_t ms)
{
    g_now += (uint64_t)ms * 1000ull;
    backdoor_tick(g_now);
}

static void reset_all(void)
{
    g_now = 1000000;
    tac_init();
    backdoor_init(sink, 0, 3);
    backdoor_set_motor_sink(motor);
    backdoor_set_encoder_provider(encoders, encoders_reset);
    mot_l = mot_r = 0;
    mot_writes = 0;
    enc_l = 1234;
    enc_r = -99;
    enc_resets = 0;
    out_clear();
}

// Acquire the lease from a clean SAFE state.
static void dev_on(void)
{
    send("dev on");
    out_clear();
}

// ---- tests -----------------------------------------------------------------

static void test_ver_and_help(void)
{
    reset_all();
    send("ver");
    CHECK(strcmp(out_line(0),
                 "=ok ver fw=0.3 iface=backdoor max_duty=600 max_ms=3000") == 0,
          "ver reports interface and rails");

    out_clear();
    send("help");
    CHECK(out_has("=ok help"), "help replies ok after its `*` lines");
    CHECK(out_count > 1, "help prints the verb list");
}

static void test_cfg_reports_configured_signs(void)
{
    reset_all();
    // Defaults before anyone publishes config: truthful, not invented.
    send("cfg");
    CHECK(strcmp(out_line(0),
                 "=ok cfg enc_left_sign=1 enc_right_sign=1 motor_left_sign=1"
                 " motor_right_sign=1 ticks_per_m=0.0 max_speed_mm_s=0") == 0,
          "cfg answers with identity defaults until config is published");

    out_clear();
    backdoor_set_config(-1, 1, 1, -1, 10000.0f, 600);
    send("cfg");
    CHECK(strcmp(out_line(0),
                 "=ok cfg enc_left_sign=-1 enc_right_sign=1 motor_left_sign=1"
                 " motor_right_sign=-1 ticks_per_m=10000.0 max_speed_mm_s=600") == 0,
          "cfg reports the compiled-in constants a bench tool needs");
    // Without this a tool cannot tell "the configured sign is correct" from
    // "the sign is +1", because `enc` counts are already sign-corrected.
}

static void test_unknown_verb_refused(void)
{
    reset_all();
    send("drive 100 100");
    CHECK(strcmp(out_line(0), "=err drive unknown_command") == 0,
          "the cockpit's motion verb is NOT reachable through the backdoor");

    out_clear();
    send("arm");
    CHECK(strcmp(out_line(0), "=err arm unknown_command") == 0,
          "closed membership: arm is not a backdoor verb");
}

static void test_gate_blocks_wiggle_without_dev(void)
{
    reset_all();
    send("wiggle 300 300 500");
    CHECK(strcmp(out_line(0), "=err wiggle dev_inactive dev on first") == 0,
          "raw motion refused without the lease");
    CHECK(mot_writes == 0, "refused wiggle never touched the motors");
    CHECK(!backdoor_wiggle_active(), "no wiggle outstanding");
}

static void test_gate_blocks_dev_when_commander_alive(void)
{
    reset_all();
    tac_note_commander_alive(g_now);       // the Pilot is talking
    send("dev on");
    CHECK(strcmp(out_line(0), "=err dev commander_present") == 0,
          "lease refused while the cockpit is live");
    CHECK(!tac_dev_active(), "lease not granted");

    // After the liveness window lapses, the same request succeeds.
    out_clear();
    g_now += (uint64_t)(LIVENESS_TIMEOUT_MS + 1) * 1000ull;
    send("dev on");
    CHECK(strcmp(out_line(0), "=ok dev") == 0,
          "lease granted once the commander has gone quiet");
    CHECK(tac_dev_active(), "lease held");
}

static void test_gate_blocks_dev_when_armed(void)
{
    reset_all();
    tac_arm();                              // FSM is ACTIVE, not SAFE
    send("dev on");
    CHECK(strcmp(out_line(0), "=err dev not_safe") == 0,
          "lease refused unless the FSM is SAFE");
}

static void test_arm_refused_while_dev_held(void)
{
    reset_all();
    dev_on();
    CHECK(tac_arm() == TAC_ERR_DEV_ACTIVE,
          "the aircraft cannot be armed while the hatch is open");
}

static void test_wiggle_applies_and_self_terminates(void)
{
    reset_all();
    dev_on();
    send("wiggle 250 -250 500");
    CHECK(strcmp(out_line(0), "=ok wiggle l=250 r=-250 ms=500") == 0,
          "reply echoes the applied command");
    CHECK(mot_l == 250 && mot_r == -250, "duty reached the motors");
    CHECK(backdoor_wiggle_active(), "wiggle outstanding");

    out_clear();
    advance_ms(499);
    CHECK(out_count == 0, "still running just before the deadline");
    CHECK(backdoor_wiggle_active(), "still outstanding");

    advance_ms(2);
    CHECK(strcmp(out_line(0), "!wiggle_done timeout") == 0,
          "announces its own expiry");
    CHECK(mot_l == 0 && mot_r == 0, "wheels stopped without being asked");
    CHECK(!backdoor_wiggle_active(), "wiggle cleared");
}

static void test_rails_clamp_rather_than_refuse(void)
{
    reset_all();
    dev_on();
    send("wiggle 9000 -9000 60000");
    CHECK(strcmp(out_line(0), "=ok wiggle l=600 r=-600 ms=3000") == 0,
          "duty and duration clamped to the bench ceiling, reply is truthful");
    CHECK(mot_l == 600 && mot_r == -600, "clamped value is what was applied");
}

static void test_wiggle_arg_validation(void)
{
    reset_all();
    dev_on();
    send("wiggle 100 100");
    CHECK(strcmp(out_line(0), "=err wiggle bad_args <left> <right> <ms>") == 0,
          "arity checked");

    out_clear();
    send("wiggle 100 abc 500");
    CHECK(strcmp(out_line(0), "=err wiggle bad_args not_a_number") == 0,
          "non-numeric refused");

    out_clear();
    send("wiggle 100 100 0");
    CHECK(strcmp(out_line(0), "=err wiggle bad_args ms_must_be_positive") == 0,
          "zero duration refused rather than latching the wheels on");
    CHECK(mot_writes == 0, "no bad request ever reached the motors");
}

static void test_commander_arrival_revokes_mid_wiggle(void)
{
    reset_all();
    dev_on();
    send("wiggle 400 400 3000");
    CHECK(mot_l == 400, "wiggle running");

    out_clear();
    tac_note_commander_alive(g_now);   // the Pilot wakes up mid-sweep
    CHECK(!tac_dev_active(), "arrival revokes the lease immediately");

    advance_ms(10);
    CHECK(strcmp(out_line(0), "!wiggle_done lease_lost") == 0,
          "the operator is told the step was truncated, not left guessing");
    CHECK(mot_l == 0 && mot_r == 0, "wheels stopped for the Pilot");
}

static void test_estop_always_honored(void)
{
    reset_all();
    dev_on();
    send("wiggle 500 500 3000");
    out_clear();

    send("estop");
    CHECK(strcmp(out_line(0), "=ok estop") == 0, "estop accepted");
    CHECK(mot_l == 0 && mot_r == 0, "estop stopped the wheels");
    CHECK(tac_state() == TacticalState::Fault, "fault latched");
    CHECK(!tac_dev_active(), "estop dropped the lease");

    // ...and it is honored with no lease at all, from a cold start.
    reset_all();
    send("estop");
    CHECK(strcmp(out_line(0), "=ok estop") == 0,
          "estop needs no dev mode, no arming, no authority");
    CHECK(tac_state() == TacticalState::Fault, "fault latched from cold");

    // A latched fault blocks the lease.
    out_clear();
    send("dev on");
    CHECK(strcmp(out_line(0), "=err dev fault_latched") == 0,
          "no bench session while a fault stands");
}

static void test_clear_fault_recovers_without_cockpit(void)
{
    reset_all();
    send("estop");
    out_clear();

    send("dev on");
    CHECK(strcmp(out_line(0), "=err dev fault_latched") == 0,
          "fault still blocks the lease before clearing");

    out_clear();
    send("clear_fault");
    CHECK(strcmp(out_line(0), "=ok clear_fault") == 0,
          "the bench can lift its own fault -- no cockpit required");
    CHECK(tac_state() == TacticalState::Safe, "back to SAFE");

    out_clear();
    send("dev on");
    CHECK(strcmp(out_line(0), "=ok dev") == 0,
          "lease grantable again after clear_fault");
}

static void test_clear_fault_refused_without_a_fault(void)
{
    reset_all();
    send("clear_fault");
    CHECK(strcmp(out_line(0), "=err clear_fault no_fault") == 0,
          "nothing to clear from a clean SAFE state");
}

static void test_dev_off_and_abort_stop_the_wheels(void)
{
    reset_all();
    dev_on();
    send("wiggle 300 300 3000");
    out_clear();
    send("dev off");
    CHECK(strcmp(out_line(0), "=ok dev") == 0, "lease released");
    CHECK(mot_l == 0 && mot_r == 0, "dev off stops any wiggle");
    CHECK(!tac_dev_active(), "lease gone");

    // Transport loss: the cable is pulled mid-wiggle.
    reset_all();
    dev_on();
    send("wiggle 300 300 3000");
    CHECK(mot_l == 300, "wiggle running");
    backdoor_abort();
    CHECK(mot_l == 0 && mot_r == 0, "unplugging the cable stops the wheels");
    CHECK(!tac_dev_active(), "abort drops the lease");
    CHECK(!backdoor_wiggle_active(), "no wiggle outstanding");
}

static void test_enc_reads_and_resets(void)
{
    reset_all();
    send("enc");
    CHECK(strcmp(out_line(0), "=ok enc left=1234 right=-99") == 0,
          "raw counts readable without the lease (it moves nothing)");

    out_clear();
    send("enc reset");
    CHECK(strcmp(out_line(0), "=ok enc left=0 right=0") == 0, "reset replies zeroed");
    CHECK(enc_resets == 1, "reset reached the driver");

    out_clear();
    send("enc");
    CHECK(strcmp(out_line(0), "=ok enc left=0 right=0") == 0, "counts really zeroed");
}

static void test_line_hygiene(void)
{
    reset_all();
    send("");                      // blank
    send("*a bench log echoed back at us");
    send("^relayed payload");
    CHECK(out_count == 0, "blank, comment and relay lines are silently ignored");

    out_clear();
    char longline[CODEC_MAX_LINE + 40];
    memset(longline, 'x', sizeof longline - 1);
    longline[sizeof longline - 1] = '\0';
    send(longline);
    CHECK(strcmp(out_line(0), "=err ? line_too_long") == 0, "overlong line reported");

    // CRLF counts as one terminator, so the next request still parses.
    out_clear();
    for (const char *p = "ver\r\n"; *p; ++p)
        backdoor_feed(*p, g_now);
    CHECK(out_count == 1, "CRLF produced exactly one reply");
}

static void test_case_insensitive_verbs(void)
{
    reset_all();
    send("DEV ON");
    CHECK(strcmp(out_line(0), "=ok dev") == 0, "verbs and args are case-insensitive");
    out_clear();
    send("WiGgLe 100 100 100");
    CHECK(strcmp(out_line(0), "=ok wiggle l=100 r=100 ms=100") == 0,
          "mixed case accepted");
}

int main(void)
{
    test_ver_and_help();
    test_cfg_reports_configured_signs();
    test_unknown_verb_refused();
    test_gate_blocks_wiggle_without_dev();
    test_gate_blocks_dev_when_commander_alive();
    test_gate_blocks_dev_when_armed();
    test_arm_refused_while_dev_held();
    test_wiggle_applies_and_self_terminates();
    test_rails_clamp_rather_than_refuse();
    test_wiggle_arg_validation();
    test_commander_arrival_revokes_mid_wiggle();
    test_estop_always_honored();
    test_clear_fault_recovers_without_cockpit();
    test_clear_fault_refused_without_a_fault();
    test_dev_off_and_abort_stop_the_wheels();
    test_enc_reads_and_resets();
    test_line_hygiene();
    test_case_insensitive_verbs();

    if (failures == 0)
        printf("test_backdoor: all checks passed\n");
    else
        printf("test_backdoor: %d FAILURES\n", failures);
    return failures != 0;
}
