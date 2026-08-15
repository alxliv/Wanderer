// Host-side tests for the cockpit handler: the full airframe stack minus the
// UART. Request lines go in through cockpit_feed(); reply and event lines
// come out through the sink. Covers the spec section 9 flows at the wire
// level, the section 5 lease rules, and the drive conversion math.
// Build: see CMakeLists.txt here, or:
//   g++ -std=c++17 -Wall -Wextra -I.. test_handler.cpp ../cockpit_handler.cpp \
//       ../cockpit_codec.cpp ../tactical.cpp -o test_handler

#include <stdio.h>
#include <string.h>

#include "cockpit_handler.h"
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

// ---- relay capture ---------------------------------------------------------

static char relay_payload[CODEC_MAX_LINE];
static int  relay_count;
static void relay(const char *payload)
{
    snprintf(relay_payload, sizeof relay_payload, "%s", payload);
    relay_count++;
}

// ---- helpers ---------------------------------------------------------------

static uint64_t g_now = 1000000;   // virtual time, us
static int32_t g_left_ticks, g_right_ticks;
static float g_psi = 0.25f, g_rate = -0.5f, g_bias = 0.01f;
static bool g_heading_valid = true;
static bool g_imu_healthy = true;

static void turn_odometry(int32_t *lt, int32_t *rt, float *vl, float *vr)
{
    *lt = g_left_ticks;
    *rt = g_right_ticks;
    *vl = *vr = 0.0f;
}

static void heading(float *psi, float *rate, float *bias, bool *valid)
{
    *psi = g_psi;
    *rate = g_rate;
    *bias = g_bias;
    *valid = g_heading_valid;
}

static void heading_zero(void) { g_psi = 0.0f; }
static bool imu_healthy(void) { return g_imu_healthy; }

static void send(const char *line)
{
    for (const char *p = line; *p; ++p)
        cockpit_feed(*p, g_now);
    cockpit_feed('\n', g_now);
}

static void fresh(void)
{
    tac_init();
    // fw 0.3, 3831 ticks/m, half-track 0.15 m (0.30 m track), wheel limit
    // 0.6 m/s -- the rover's real geometry and DEFAULT_MAX_SPEED_MM_S.
    cockpit_init(sink, 0, 3, 3831.0f, 0.15f, 0.6f);
    cockpit_set_turn_config(0.5235988f, 0.0349066f, 0.2f);
    cockpit_set_move_config(2.0f, 0.5f, 0.1f, 0.3f, 0.5f);
    cockpit_set_motor_config(1000, 841, 80, 40);
    cockpit_set_odometry_provider(turn_odometry);
    cockpit_set_heading_provider(heading, heading_zero);
    cockpit_set_imu_healthy_provider(imu_healthy);
    out_clear();
    relay_count = 0;
    g_now = 1000000;
    g_left_ticks = g_right_ticks = 0;
    g_psi = 0.25f;
    g_rate = -0.5f;
    g_bias = 0.01f;
    g_heading_valid = true;
    g_imu_healthy = true;
}

#define EXPECT(idx, want) CHECK(strcmp(out_line(idx), want) == 0, \
                                "expected \"" want "\"")

// ---- tests -----------------------------------------------------------------

static void test_handshake_and_motion(void)   // spec section 9, first example
{
    fresh();
    send("ping");            EXPECT(0, "=ok ping");
    send("get_version");     EXPECT(1, "=ok get_version fw=0.3");
    send("get_state");       EXPECT(2, "=ok get_state state=SAFE");
    send("get_geometry");    EXPECT(3, "=ok get_geometry tpm=3831.000 track=0.300");
    out_clear();

    send("arm");
    // Reply and its event, in whichever order they landed (spec section 2:
    // no ordering guarantee) -- with this implementation the event is
    // emitted from inside tac_arm(), so it precedes the reply.
    CHECK(out_count == 2, "arm produces event + reply");
    EXPECT(0, "!state from=SAFE to=ACTIVE");
    EXPECT(1, "=ok arm");
    out_clear();

    send("drive 0.300 0.000");   EXPECT(0, "=ok drive");
    CHECK(tac_target_left() == 300 && tac_target_right() == 300,
          "straight drive: both wheels 300 mm/s");
    send("drive 0.200 0.500");
    // Positive omega turns right, so left is the outer wheel:
    // left = 0.2 + 0.5*0.15 = 0.275; right = 0.2 - 0.075 = 0.125
    CHECK(tac_target_left() == 275 && tac_target_right() == 125,
          "arc drive converts with half-track");
    out_clear();

    send("stop");            EXPECT(0, "=ok stop");
    CHECK(tac_state() == TacticalState::Active, "stop stays ACTIVE");
    CHECK(tac_target_left() == 0, "stop zeroes velocity");
    out_clear();

    send("disarm");
    EXPECT(0, "!state from=ACTIVE to=SAFE");
    EXPECT(1, "=ok disarm");
}

static void test_refusals_and_unknown(void)
{
    fresh();
    send("drive 0.3 0.0");   EXPECT(0, "=err drive not_armed");
    send("stop");            EXPECT(1, "=err stop not_armed");
    send("clear_fault");     EXPECT(2, "=err clear_fault no_fault");
    send("mve 1 2");         EXPECT(3, "=err ? unknown_command mve");
    send("drive 0.3 x");     EXPECT(4, "=err drive bad_args expected 2 numbers");
    send("drive 0.3");       EXPECT(5, "=err drive bad_args expected 2 numbers");
    CHECK(tac_state() == TacticalState::Safe, "nothing changed state");
}

static void test_estop_latch_and_clear(void)   // spec section 9 example
{
    fresh();
    send("arm");
    out_clear();
    send("estop");
    EXPECT(0, "!fault code=ESTOP");
    EXPECT(1, "!state from=ACTIVE to=FAULT");
    EXPECT(2, "=ok estop");
    out_clear();
    send("arm");             EXPECT(0, "=err arm fault_latched");
    send("get_state");       EXPECT(1, "=ok get_state state=FAULT fault=ESTOP");
    out_clear();
    send("clear_fault");
    EXPECT(0, "!state from=FAULT to=SAFE");
    EXPECT(1, "=ok clear_fault");
}

static void test_lease_rules(void)   // spec section 5
{
    // A known-but-refused request refreshes the lease...
    fresh();
    send("arm");
    send("drive 0.3 0.0");
    for (int i = 0; i < 10; ++i) {
        g_now += 200000;               // 200 ms steps, inside the 750 ms window
        send("clear_fault");           // refused (=err no_fault) but KNOWN
        tac_tick(g_now);
    }
    CHECK(tac_state() == TacticalState::Active,
          "refused-but-known requests keep the lease alive");

    // ...unknown verbs, relay lines and comments do not.
    fresh();
    cockpit_set_relay_sink(relay);
    send("arm");
    send("drive 0.3 0.0");
    out_clear();
    for (int i = 0; i < 6; ++i) {
        g_now += 200000;
        send("bogus 1");               // unknown verb
        send("^GOAL waypoint 1 2");    // relay: opaque, no lease
        send("*just a comment");
        tac_tick(g_now);
    }
    CHECK(tac_state() == TacticalState::Fallback,
          "unknown/relay/comment lines do not refresh the lease");
    bool saw_fallback = false;
    for (int i = 0; i < out_count; ++i)
        if (strcmp(out_line(i), "!state from=ACTIVE to=FALLBACK") == 0)
            saw_fallback = true;
    CHECK(saw_fallback, "deadman transition reported as !state");
    CHECK(relay_count == 6 && strcmp(relay_payload, "GOAL waypoint 1 2") == 0,
          "relay payloads delivered verbatim");

    // A fresh drive is the only resume, and it reports.
    out_clear();
    send("drive 0.100 0.000");
    EXPECT(0, "!state from=FALLBACK to=ACTIVE");
    EXPECT(1, "=ok drive");
}

// The vehicle limit: a request beyond what the wheels can deliver is scaled as
// a PAIR, so the commanded arc survives and only the speed drops.
static void test_drive_saturation(void)
{
    fresh();
    send("arm");
    out_clear();

    // Exactly at the limit is not scaled, and the reply stays bare.
    send("drive 0.600 0.000");
    EXPECT(0, "=ok drive");
    CHECK(tac_target_left() == 600 && tac_target_right() == 600,
          "a request at the limit passes through untouched");
    out_clear();

    // A positive omega is a turn to the RIGHT, so the LEFT wheel is the fast
    // one: left = 0.65, right = 0.35. Only the left wheel is over, but BOTH
    // scale by 0.6/0.65. Clipping the left wheel alone would give 600/350 and
    // turn the commanded 0.50 m turn radius into 0.57 m -- silently.
    send("drive 0.500 1.000");
    EXPECT(0, "=ok drive lin=0.462 omega=0.923");
    CHECK(tac_target_left() == 600 && tac_target_right() == 323,
          "an over-limit pair scales together");
    CHECK(tac_target_right() * 1000 / tac_target_left() == 538,
          "wheel ratio 0.35/0.65 survives, so the arc does too");
    out_clear();

    // Pure spin: the fastest this geometry can yaw is 2*0.6/0.30 = 4 rad/s.
    send("drive 0.000 10.000");
    EXPECT(0, "=ok drive lin=0.000 omega=4.000");
    CHECK(tac_target_left() == 600 && tac_target_right() == -600,
          "spin saturates symmetrically");
    out_clear();

    // Absurd-but-finite input lands on the VEHICLE limit, not on the int16
    // guard in wheel_mm_s().
    send("drive 1e30 0");
    EXPECT(0, "=ok drive lin=0.600 omega=0.000");
    CHECK(tac_target_left() == 600 && tac_target_right() == 600,
          "huge finite request lands on the vehicle limit");
    out_clear();

    // Finite inputs whose MIX overflows float are refused, not scaled:
    // 3e38 + 3e38*0.15 exceeds FLT_MAX.
    send("drive 3.0e38 3.0e38");
    EXPECT(0, "=err drive bad_args out of range");
    CHECK(tac_target_left() == 600, "a refused drive leaves targets alone");
    out_clear();

    // Limiting is opt-in: <= 0 leaves only the int16 range guard.
    tac_init();
    cockpit_init(sink, 0, 3, 3831.0f, 0.15f, 0.0f);
    send("arm");
    out_clear();
    send("drive 5.000 0.000");
    EXPECT(0, "=ok drive");
    CHECK(tac_target_left() == 5000 && tac_target_right() == 5000,
          "max_wheel_m_s <= 0 disables limiting");
}

static void odom_fixture(int32_t *lt, int32_t *rt, float *vl, float *vr)
{
    *lt = 15320; *rt = 15294; *vl = 0.298f; *vr = 0.301f;
}

static void test_odometry_and_overflow(void)
{
    fresh();
    send("get_odometry");
    EXPECT(0, "=ok get_odometry lt=0 rt=0 vl=0.000 vr=0.000");
    cockpit_set_odometry_provider(odom_fixture);
    send("get_odometry");
    EXPECT(1, "=ok get_odometry lt=15320 rt=15294 vl=0.298 vr=0.301");
    out_clear();

    for (int i = 0; i < 200; ++i)
        cockpit_feed('x', g_now);
    cockpit_feed('\n', g_now);
    EXPECT(0, "=err ? line_too_long");
    send("ping");
    EXPECT(1, "=ok ping");   // clean line right after the overflow
}

static void test_heading_queries(void)
{
    fresh();
    send("get_heading");
    EXPECT(0, "=ok get_heading psi=0.250000 rate=-0.500000 bias=0.010000 valid=1");

    out_clear();
    send("zero_heading");
    EXPECT(0, "=ok zero_heading");
    CHECK(g_psi == 0.0f, "zero_heading reached the estimator");

    out_clear();
    g_heading_valid = false;
    send("zero_heading");
    EXPECT(0, "=err zero_heading imu_not_ready");
}

static void test_motor_config_query(void)
{
    fresh();
    send("get_motor_config");
    EXPECT(0, "=ok get_motor_config lgain=1000 rgain=841 ldead=80 rdead=40");
}

static void test_turn_procedure(void)
{
    fresh();
    send("arm");
    out_clear();
    send("proc turn 1.570796 0.100");
    EXPECT(0, "=ok proc name=turn lin=0.100 timeout=6.000");
        CHECK(tac_target_left() == 179 && tac_target_right() == 21,
            "turn procedure owns curved wheel targets");

        out_clear();
        // Wild encoder motion does not complete an IMU-owned turn.
        g_left_ticks = 100000;
        g_right_ticks = -100000;
        cockpit_tick(g_now + 100000);
        CHECK(out_count == 0, "encoder heading cannot finish an IMU turn");

        g_psi = 1.80f;  // +1.55 rad from the 0.25 rad start heading
        cockpit_tick(g_now + 3000000);
        EXPECT(0, "!proc name=turn outcome=DONE");
        CHECK(tac_target_left() == 100 && tac_target_right() == 100,
            "completed turn restores straight linear motion");

        out_clear();
        send("proc turn -1.570796 -0.100");
        out_clear();
        send("abort");
        EXPECT(0, "!proc name=turn outcome=ABORTED reason=command");
        EXPECT(1, "=ok abort");
        CHECK(tac_target_left() == -100 && tac_target_right() == -100,
            "abort restores straight reverse motion");

        out_clear();
        send("proc turn 1.570796 0.000");
        out_clear();
        g_now += 800000;
        tac_tick(g_now);
        cockpit_tick(g_now);
        EXPECT(0, "!state from=ACTIVE to=FALLBACK");
        EXPECT(1, "!proc name=turn outcome=ABORTED reason=deadman");
}

static void test_turn_imu_guards(void)
{
    fresh();
    send("arm");
    out_clear();
    g_heading_valid = false;
    send("proc turn 1.570796 0.000");
    EXPECT(0, "=err proc imu_not_ready");

    out_clear();
    g_heading_valid = true;
    g_imu_healthy = false;
    send("proc turn 1.570796 0.000");
    EXPECT(0, "=err proc imu_not_ready");

    out_clear();
    g_imu_healthy = true;
    send("proc turn 1.570796 0.000");
    EXPECT(0, "=ok proc name=turn lin=0.000 timeout=6.000");
    out_clear();
    g_imu_healthy = false;
    cockpit_tick(g_now + 100000);
    EXPECT(0, "!proc name=turn outcome=ABORTED reason=imu_stale");
    CHECK(tac_target_left() == 0 && tac_target_right() == 0,
          "stale IMU stops rather than continuing the turn");
}

static void test_move_procedure(void)
{
    fresh();
    send("arm");
    out_clear();
    send("proc move 1.000 0.200");
    EXPECT(0, "=ok proc name=move lin=0.200 timeout=8.000");
    CHECK(tac_target_left() == 200 && tac_target_right() == 200,
          "move starts straight at the requested speed");

    // A positive current heading is an unwanted right turn. The corrective
    // omega must slow the left wheel and speed the right wheel to steer left.
    out_clear();
    g_psi = 0.35f;
    g_rate = 0.0f;
    cockpit_tick(g_now + 10000);
    CHECK(tac_target_left() < 200 && tac_target_right() > 200,
          "move PID mixes heading correction into the wheel pair");

    out_clear();
    g_left_ticks = g_right_ticks = 3831;
    cockpit_tick(g_now + 20000);
    EXPECT(0, "!proc name=move outcome=DONE");
    CHECK(tac_target_left() == 0 && tac_target_right() == 0,
          "completed move stops rather than restoring prior motion");

    fresh();
    send("arm");
    out_clear();
    g_heading_valid = false;
    send("proc move 1.000 0.200");
    EXPECT(0, "=err proc imu_not_ready");

    out_clear();
    g_heading_valid = true;
    send("proc move 1.000 -0.200");
    EXPECT(0, "=err proc bad_args distance and linear must agree");

    out_clear();
    send("proc move 1.000 0.200");
    out_clear();
    g_imu_healthy = false;
    cockpit_tick(g_now + 10000);
    EXPECT(0, "!proc name=move outcome=ABORTED reason=imu_stale");
    CHECK(tac_target_left() == 0 && tac_target_right() == 0,
          "stale IMU stops an active move");
}

int main(void)
{
    test_handshake_and_motion();
    test_refusals_and_unknown();
    test_estop_latch_and_clear();
    test_lease_rules();
    test_drive_saturation();
    test_odometry_and_overflow();
    test_heading_queries();
    test_motor_config_query();
    test_turn_procedure();
    test_turn_imu_guards();
    test_move_procedure();
    if (failures == 0)
        printf("OK: cockpit handler wire-level tests pass\n");
    else
        printf("%d FAILURES\n", failures);
    return failures == 0 ? 0 : 1;
}
