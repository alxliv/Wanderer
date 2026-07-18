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
                  float half_track_m)
{
    s_sink = sink;
    s_fwMajor = fw_major;
    s_fwMinor = fw_minor;
    s_halfTrackM = half_track_m;
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

// Body velocities (m/s, rad/s) -> wheel targets (mm/s), differential drive.
static int16_t wheel_mm_s(float m_s)
{
    float v = roundf(m_s * 1000.0f);
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
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
        float lin, ang;
        if (n != 3 || !codec_parse_f32(tok[1], &lin)
                   || !codec_parse_f32(tok[2], &ang)) {
            reply_err("drive", "bad_args", "expected 2 numbers");
            return;
        }
        int16_t left  = wheel_mm_s(lin - ang * s_halfTrackM);
        int16_t right = wheel_mm_s(lin + ang * s_halfTrackM);
        reply_rc("drive", tac_drive(left, right));
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
