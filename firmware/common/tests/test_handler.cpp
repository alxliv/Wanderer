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

static void send(const char *line)
{
    for (const char *p = line; *p; ++p)
        cockpit_feed(*p, g_now);
    cockpit_feed('\n', g_now);
}

static void fresh(void)
{
    tac_init();
    cockpit_init(sink, 0, 3, 0.15f);   // fw 0.3, half-track 0.15 m
    out_clear();
    relay_count = 0;
    g_now = 1000000;
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
    // left = 0.2 - 0.5*0.15 = 0.125; right = 0.2 + 0.075 = 0.275
    CHECK(tac_target_left() == 125 && tac_target_right() == 275,
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

int main(void)
{
    test_handshake_and_motion();
    test_refusals_and_unknown();
    test_estop_latch_and_clear();
    test_lease_rules();
    test_odometry_and_overflow();
    if (failures == 0)
        printf("OK: cockpit handler wire-level tests pass\n");
    else
        printf("%d FAILURES\n", failures);
    return failures == 0 ? 0 : 1;
}
