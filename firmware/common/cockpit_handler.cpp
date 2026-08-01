#include "cockpit_handler.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "cockpit_codec.h"
#include "tactical.h"

// ---- module state ----------------------------------------------------------

static cockpit_sink    s_sink;
static cockpit_sink    s_relay;
static cockpit_odom_fn s_odom;
static uint8_t         s_fwMajor, s_fwMinor;
static float           s_halfTrackM;
static float           s_maxWheelMs;
static LineAssembler   s_asm;

static void emit(const char *line)
{
    if (s_sink)
        s_sink(line);
}

// Every FSM transition, whatever the cause. `!fault` first when latching
// (the code is latched before this callback fires -- tactical.cpp).
static void on_change_state(TacticalState from, TacticalState to)
{
    char line[CODEC_MAX_LINE];
    if (to == TacticalState::Fault) {
        if (codec_format_fault_event(line, sizeof line,
                                     tac_fault_name(tac_fault_code())) > 0)
            emit(line);
    }
    if (codec_format_state_event(line, sizeof line,
                                 tac_state_name(from), tac_state_name(to)) > 0)
        emit(line);
}

void cockpit_init(cockpit_sink sink, uint8_t fw_major, uint8_t fw_minor,
                  float half_track_m, float max_wheel_m_s)
{
    s_sink = sink;
    s_fwMajor = fw_major;
    s_fwMinor = fw_minor;
    s_halfTrackM = half_track_m;
    s_maxWheelMs = max_wheel_m_s;
    s_relay = NULL;
    s_odom = NULL;
    line_asm_init(&s_asm);
    tac_set_change_state_callback(on_change_state);
}

void cockpit_set_odometry_provider(cockpit_odom_fn fn) { s_odom = fn; }
void cockpit_set_relay_sink(cockpit_sink fn)           { s_relay = fn; }

// ---- replies ---------------------------------------------------------------

static void reply_rc(const char *verb, int rc)
{
    char line[CODEC_MAX_LINE];
    int n = (rc == TAC_OK)
        ? codec_format_ok(line, sizeof line, verb, NULL)
        : codec_format_err(line, sizeof line, verb, tac_strerror(rc), NULL);
    if (n > 0)
        emit(line);
}

static void reply_ok_fields(const char *verb, const char *fields)
{
    char line[CODEC_MAX_LINE];
    if (codec_format_ok(line, sizeof line, verb, fields) > 0)
        emit(line);
}

static void reply_err(const char *verb, const char *reason, const char *detail)
{
    char line[CODEC_MAX_LINE];
    if (codec_format_err(line, sizeof line, verb, reason, detail) > 0)
        emit(line);
}

// One wheel's velocity (m/s) -> the tactical layer's units (mm/s).
//
// The bounds here are int16_t's, NOT the vehicle's: casting a float that does
// not fit the destination type is undefined behavior, so the value is made
// castable before the cast. NaN is rejected first because it fails every
// comparison and would slip past both clamps -- codec_parse_f32 refuses
// non-finite input so one cannot arrive from the wire, but this function must
// be total. The VEHICLE's limit is a separate concern, applied by
// drive_scale() below while there is still a reply channel to report it.
static int16_t wheel_mm_s(float m_s)
{
    float v = roundf(m_s * 1000.0f);
    if (isnan(v))
        return 0;
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
}

// The vehicle's own limit, applied to the wheel PAIR. Returns the factor to
// multiply both wheels by (1.0 when the request already fits).
//
// Wheel targets beyond what the drivetrain can deliver are scaled down
// TOGETHER, never clipped apart. Clipping one wheel changes the DIFFERENCE
// between the two, and that difference is the turn: the rover would quietly
// drive a wider arc than commanded and bias the Pilot's dead reckoning.
// Scaling both by one factor leaves left:right -- and so the commanded turn
// radius -- intact, and simply traverses the same arc more slowly.
static float drive_scale(float left_m_s, float right_m_s)
{
    if (s_maxWheelMs <= 0.0f)
        return 1.0f;                  // limiting not configured by the caller
    float l = fabsf(left_m_s), r = fabsf(right_m_s);
    float peak = (l > r) ? l : r;
    if (peak <= s_maxWheelMs)
        return 1.0f;
    return s_maxWheelMs / peak;
}

// ---- request dispatch ------------------------------------------------------

static void handle_request(char *line, uint64_t now_us)
{
    char *tok[CODEC_MAX_TOKENS];
    int n = codec_tokenize(line, tok, CODEC_MAX_TOKENS);
    if (n <= 0) {
        reply_err("?", "bad_args", NULL);
        return;
    }
    const char *verb = tok[0];

    // The spec's lease rule (section 5): any KNOWN verb refreshes, even if
    // the command is then refused. Unknown verbs do not.
    static const char *KNOWN[] = {
        "ping", "arm", "disarm", "estop", "clear_fault",
        "drive", "stop", "get_state", "get_odometry", "get_version",
    };
    bool known = false;
    for (unsigned i = 0; i < sizeof KNOWN / sizeof KNOWN[0]; ++i)
        if (codec_token_eq(verb, KNOWN[i]))
            known = true;
    if (!known) {
        reply_err("?", "unknown_command", verb);
        return;
    }
    tac_note_commander_alive(now_us);

    if (codec_token_eq(verb, "ping")) {
        reply_rc("ping", TAC_OK);
    } else if (codec_token_eq(verb, "arm")) {
        reply_rc("arm", tac_arm());
    } else if (codec_token_eq(verb, "disarm")) {
        reply_rc("disarm", tac_disarm());
    } else if (codec_token_eq(verb, "estop")) {
        reply_rc("estop", tac_estop());
    } else if (codec_token_eq(verb, "clear_fault")) {
        // ESTOP's condition is definitionally gone once commanded away
        // (spec section 6). Tier 3 faults will plug a real check in here.
        reply_rc("clear_fault", tac_clear_fault(true));
    } else if (codec_token_eq(verb, "stop")) {
        reply_rc("stop", tac_stop());
    } else if (codec_token_eq(verb, "drive")) {
        float lin_m_s, omega_rad_s;
        if (n != 3 || !codec_parse_f32(tok[1], &lin_m_s)
                   || !codec_parse_f32(tok[2], &omega_rad_s)) {
            reply_err("drive", "bad_args", "expected 2 numbers");
            return;
        }
        float left  = lin_m_s - omega_rad_s * s_halfTrackM;
        float right = lin_m_s + omega_rad_s * s_halfTrackM;
        // Both inputs are finite (codec_parse_f32 guarantees it), but their
        // sum can still overflow at the extremes of float range. Refuse rather
        // than scale an infinity -- max/inf would be 0, and inf*0 is NaN.
        if (!isfinite(left) || !isfinite(right)) {
            reply_err("drive", "bad_args", "out of range");
            return;
        }
        float scale = drive_scale(left, right);
        int rc = tac_drive(wheel_mm_s(left * scale), wheel_mm_s(right * scale));
        if (rc == TAC_OK && scale < 1.0f) {
            // Spec section 3: report what was actually applied, and ONLY when
            // it differs from the request -- the fields' presence IS the
            // saturation signal, so the common case stays a bare `=ok drive`
            // and the streaming reply does not grow.
            char fields[48];
            snprintf(fields, sizeof fields, "lin=%.3f omega=%.3f",
                     (double)(lin_m_s * scale), (double)(omega_rad_s * scale));
            reply_ok_fields("drive", fields);
        } else {
            reply_rc("drive", rc);
        }
    } else if (codec_token_eq(verb, "get_state")) {
        char fields[64];
        if (tac_state() == TacticalState::Fault)
            snprintf(fields, sizeof fields, "state=%s fault=%s",
                     tac_state_name(tac_state()),
                     tac_fault_name(tac_fault_code()));
        else
            snprintf(fields, sizeof fields, "state=%s",
                     tac_state_name(tac_state()));
        reply_ok_fields("get_state", fields);
    } else if (codec_token_eq(verb, "get_odometry")) {
        int32_t lt = 0, rt = 0;
        float vl = 0.0f, vr = 0.0f;
        if (s_odom)
            s_odom(&lt, &rt, &vl, &vr);
        char fields[80];
        snprintf(fields, sizeof fields, "lt=%ld rt=%ld vl=%.3f vr=%.3f",
                 (long)lt, (long)rt, (double)vl, (double)vr);
        reply_ok_fields("get_odometry", fields);
    } else if (codec_token_eq(verb, "get_version")) {
        char fields[24];
        snprintf(fields, sizeof fields, "fw=%u.%u", s_fwMajor, s_fwMinor);
        reply_ok_fields("get_version", fields);
    }
}

// ---- byte pump -------------------------------------------------------------

void cockpit_feed(char c, uint64_t now_us)
{
    if (!line_asm_feed(&s_asm, c))
        return;
    if (s_asm.overflow) {
        reply_err("?", "line_too_long", NULL);
        return;
    }
    const char *payload = NULL;
    switch (codec_classify(s_asm.buf, &payload)) {
    case LINE_REQUEST:
        handle_request(s_asm.buf, now_us);
        break;
    case LINE_RELAY:                 // opaque; no reply, no lease refresh
        if (s_relay)
            s_relay(payload);
        break;
    case LINE_IGNORE:
        break;
    }
}
