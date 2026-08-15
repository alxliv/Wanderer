#include "cockpit_handler.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "angle.h"
#include "cockpit_codec.h"
#include "tactical.h"

// ---- module state ----------------------------------------------------------

static cockpit_sink    s_sink;
static cockpit_sink    s_relay;
static cockpit_odom_fn s_odom;
static cockpit_heading_fn s_heading;
static cockpit_heading_zero_fn s_headingZero;
static cockpit_imu_healthy_fn s_imuHealthy;
static uint8_t         s_fwMajor, s_fwMinor;
static float           s_ticksPerMeter;
static float           s_halfTrackM;
static float           s_maxWheelMs;
static float           s_turnRateRadS;
static float           s_turnOvershootRad;
static float           s_turnLinearLimitMs;
static float           s_headingKp, s_headingKi, s_headingKd;
static float           s_headingIntegralMaxRadS;
static float           s_headingOmegaMaxRadS;
static uint16_t        s_motorLeftGain, s_motorRightGain;
static uint16_t        s_motorLeftDeadband, s_motorRightDeadband;
static bool            s_turnActive;
static float           s_turnAngleRad;
static float           s_turnLinearMs;
static float           s_turnLastPsiRad;
static float           s_turnProgressRad;
static uint64_t        s_turnDeadlineUs;
static bool            s_moveActive;
static float           s_moveDistanceM;
static float           s_moveLinearMs;
static int32_t         s_moveStartLeft, s_moveStartRight;
static float           s_movePsiRefRad;
static float           s_moveHeadingIntegral;
static uint64_t        s_moveLastControlUs;
static uint64_t        s_moveDeadlineUs;
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
                  float ticks_per_meter, float half_track_m,
                  float max_wheel_m_s)
{
    s_sink = sink;
    s_fwMajor = fw_major;
    s_fwMinor = fw_minor;
    s_ticksPerMeter = ticks_per_meter;
    s_halfTrackM = half_track_m;
    s_maxWheelMs = max_wheel_m_s;
    s_turnRateRadS = 0.0f;
    s_turnOvershootRad = 0.0f;
    s_turnLinearLimitMs = 0.0f;
    s_headingKp = s_headingKi = s_headingKd = 0.0f;
    s_headingIntegralMaxRadS = 0.0f;
    s_headingOmegaMaxRadS = 0.0f;
    s_motorLeftGain = s_motorRightGain = 0;
    s_motorLeftDeadband = s_motorRightDeadband = 0;
    s_turnActive = false;
    s_moveActive = false;
    s_relay = NULL;
    s_odom = NULL;
    s_heading = NULL;
    s_headingZero = NULL;
    s_imuHealthy = NULL;
    line_asm_init(&s_asm);
    tac_set_change_state_callback(on_change_state);
}

void cockpit_set_odometry_provider(cockpit_odom_fn fn) { s_odom = fn; }
void cockpit_set_heading_provider(cockpit_heading_fn fn,
                                  cockpit_heading_zero_fn zero_fn)
{
    s_heading = fn;
    s_headingZero = zero_fn;
}
void cockpit_set_imu_healthy_provider(cockpit_imu_healthy_fn fn)
{
    s_imuHealthy = fn;
}
void cockpit_set_relay_sink(cockpit_sink fn)           { s_relay = fn; }

void cockpit_set_turn_config(float rate_rad_s, float overshoot_rad,
                             float linear_limit_m_s)
{
    s_turnRateRadS = rate_rad_s;
    s_turnOvershootRad = overshoot_rad;
    s_turnLinearLimitMs = linear_limit_m_s;
}

void cockpit_set_move_config(float heading_kp, float heading_ki,
                             float heading_kd, float integral_max_rad_s,
                             float omega_max_rad_s)
{
    s_headingKp = heading_kp;
    s_headingKi = heading_ki;
    s_headingKd = heading_kd;
    s_headingIntegralMaxRadS = integral_max_rad_s;
    s_headingOmegaMaxRadS = omega_max_rad_s;
}

void cockpit_set_motor_config(uint16_t left_gain_permille,
                              uint16_t right_gain_permille,
                              uint16_t left_deadband_permille,
                              uint16_t right_deadband_permille)
{
    s_motorLeftGain = left_gain_permille;
    s_motorRightGain = right_gain_permille;
    s_motorLeftDeadband = left_deadband_permille;
    s_motorRightDeadband = right_deadband_permille;
}

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

static void emit_proc_event(const char *name, const char *outcome,
                            const char *reason)
{
    char line[CODEC_MAX_LINE];
    if (reason)
        snprintf(line, sizeof line,
                 "!proc name=%s outcome=%s reason=%s", name, outcome, reason);
    else
        snprintf(line, sizeof line, "!proc name=%s outcome=%s", name, outcome);
    emit(line);
}

static void turn_finish(const char *outcome, const char *reason,
                        bool restore_linear)
{
    if (!s_turnActive)
        return;
    s_turnActive = false;
    if (restore_linear && tac_state() == TacticalState::Active) {
        const int16_t mm_s = wheel_mm_s(s_turnLinearMs);
        tac_drive(mm_s, mm_s);
    } else if (tac_state() == TacticalState::Active) {
        tac_drive(0, 0);
    }
    emit_proc_event("turn", outcome, reason);
}

static void move_finish(const char *outcome, const char *reason)
{
    if (!s_moveActive)
        return;
    s_moveActive = false;
    if (tac_state() == TacticalState::Active)
        tac_drive(0, 0);
    emit_proc_event("move", outcome, reason);
}

static int64_t wrapping_tick_delta(int32_t current, int32_t start)
{
    const uint32_t raw = (uint32_t)current - (uint32_t)start;
    if (raw <= (uint32_t)INT32_MAX)
        return (int64_t)raw;
    return (int64_t)raw - (INT64_C(1) << 32);
}

static bool turn_imu_ready(float *psi_rad)
{
    if (!s_heading || !s_imuHealthy || !s_imuHealthy())
        return false;
    float psi, rate, bias;
    bool valid = false;
    s_heading(&psi, &rate, &bias, &valid);
    if (!valid)
        return false;
    if (psi_rad)
        *psi_rad = psi;
    return true;
}

static int start_turn(float angle_rad, float linear_m_s, uint64_t now_us,
                      float psi_rad)
{
    if (!(s_halfTrackM > 0.0f) || !(s_turnRateRadS > 0.0f))
        return TAC_ERR_NOT_SAFE;

    if (s_turnLinearLimitMs > 0.0f
            && fabsf(linear_m_s) > s_turnLinearLimitMs)
        linear_m_s = copysignf(s_turnLinearLimitMs, linear_m_s);

    const float omega = copysignf(s_turnRateRadS, angle_rad);
    const float left = linear_m_s + omega * s_halfTrackM;
    const float right = linear_m_s - omega * s_halfTrackM;
    const float scale = drive_scale(left, right);
    const int rc = tac_drive(wheel_mm_s(left * scale),
                             wheel_mm_s(right * scale));
    if (rc != TAC_OK)
        return rc;

    s_turnActive = true;
    s_turnAngleRad = angle_rad;
    s_turnLinearMs = linear_m_s;
    s_turnLastPsiRad = psi_rad;
    s_turnProgressRad = 0.0f;
    const float applied_rate = s_turnRateRadS * scale;
    const uint64_t duration_us = (uint64_t)(fabsf(angle_rad) / applied_rate
                                           * 1000000.0f);
    s_turnDeadlineUs = now_us + duration_us + UINT64_C(3000000);
    return TAC_OK;
}

static int start_move(float distance_m, float linear_m_s, uint64_t now_us,
                      float psi_rad)
{
    if (!s_odom || !(s_ticksPerMeter > 0.0f) || !(s_halfTrackM > 0.0f)
            || !(s_headingOmegaMaxRadS > 0.0f))
        return TAC_ERR_NOT_SAFE;

    const float speed_scale = drive_scale(linear_m_s, linear_m_s);
    linear_m_s *= speed_scale;
    const int rc = tac_drive(wheel_mm_s(linear_m_s), wheel_mm_s(linear_m_s));
    if (rc != TAC_OK)
        return rc;

    float vl, vr;
    s_odom(&s_moveStartLeft, &s_moveStartRight, &vl, &vr);
    s_moveActive = true;
    s_moveDistanceM = distance_m;
    s_moveLinearMs = linear_m_s;
    s_movePsiRefRad = psi_rad;
    s_moveHeadingIntegral = 0.0f;
    s_moveLastControlUs = now_us;
    s_moveDeadlineUs = now_us + (uint64_t)(fabsf(distance_m / linear_m_s)
                                            * 1000000.0f)
                        + UINT64_C(3000000);
    return TAC_OK;
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
        "get_geometry", "get_motor_config", "get_heading", "zero_heading", "proc", "abort",
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
        turn_finish("ABORTED", "disarm", false);
        move_finish("ABORTED", "disarm");
        reply_rc("disarm", tac_disarm());
    } else if (codec_token_eq(verb, "estop")) {
        turn_finish("ABORTED", "estop", false);
        move_finish("ABORTED", "estop");
        reply_rc("estop", tac_estop());
    } else if (codec_token_eq(verb, "clear_fault")) {
        // ESTOP's condition is definitionally gone once commanded away
        // (spec section 6). Tier 3 faults will plug a real check in here.
        reply_rc("clear_fault", tac_clear_fault(true));
    } else if (codec_token_eq(verb, "stop")) {
        turn_finish("ABORTED", "stop", false);
        move_finish("ABORTED", "stop");
        reply_rc("stop", tac_stop());
    } else if (codec_token_eq(verb, "drive")) {
        if (s_turnActive || s_moveActive) {
            reply_err("drive", "busy", "procedure active");
            return;
        }
        float lin_m_s, omega_rad_s;
        if (n != 3 || !codec_parse_f32(tok[1], &lin_m_s)
                   || !codec_parse_f32(tok[2], &omega_rad_s)) {
            reply_err("drive", "bad_args", "expected 2 numbers");
            return;
        }
        float left  = lin_m_s + omega_rad_s * s_halfTrackM;
        float right = lin_m_s - omega_rad_s * s_halfTrackM;
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
    } else if (codec_token_eq(verb, "get_heading")) {
        float psi = 0.0f, rate = 0.0f, bias = 0.0f;
        bool valid = false;
        if (s_heading)
            s_heading(&psi, &rate, &bias, &valid);
        char fields[112];
        snprintf(fields, sizeof fields,
                 "psi=%.6f rate=%.6f bias=%.6f valid=%d",
                 (double)psi, (double)rate, (double)bias, valid ? 1 : 0);
        reply_ok_fields("get_heading", fields);
    } else if (codec_token_eq(verb, "zero_heading")) {
        float psi, rate, bias;
        bool valid = false;
        if (s_heading)
            s_heading(&psi, &rate, &bias, &valid);
        if (!valid || !s_headingZero) {
            reply_err("zero_heading", "imu_not_ready", NULL);
            return;
        }
        s_headingZero();
        reply_rc("zero_heading", TAC_OK);
    } else if (codec_token_eq(verb, "get_version")) {
        char fields[24];
        snprintf(fields, sizeof fields, "fw=%u.%u", s_fwMajor, s_fwMinor);
        reply_ok_fields("get_version", fields);
    } else if (codec_token_eq(verb, "get_geometry")) {
        char fields[48];
        snprintf(fields, sizeof fields, "tpm=%.3f track=%.3f",
                 (double)s_ticksPerMeter, (double)(2.0f * s_halfTrackM));
        reply_ok_fields("get_geometry", fields);
    } else if (codec_token_eq(verb, "get_motor_config")) {
        char fields[80];
        snprintf(fields, sizeof fields, "lgain=%u rgain=%u ldead=%u rdead=%u",
                 (unsigned)s_motorLeftGain, (unsigned)s_motorRightGain,
                 (unsigned)s_motorLeftDeadband, (unsigned)s_motorRightDeadband);
        reply_ok_fields("get_motor_config", fields);
    } else if (codec_token_eq(verb, "proc")) {
        float psi_rad;
        float value, linear_m_s;
        const bool is_turn = n >= 2 && codec_token_eq(tok[1], "turn");
        const bool is_move = n >= 2 && codec_token_eq(tok[1], "move");
        if (!is_turn && !is_move) {
            reply_err("proc", "bad_args", "expected turn or move");
            return;
        }
        if (n != 4 || !codec_parse_f32(tok[2], &value)
                   || !codec_parse_f32(tok[3], &linear_m_s)) {
            reply_err("proc", "bad_args", "expected turn/move value linear");
            return;
        }
        if (!turn_imu_ready(&psi_rad)) {
            reply_err("proc", "imu_not_ready", NULL);
            return;
        }
        if (is_turn) {
            if (value == 0.0f) {
                reply_err("proc", "bad_args", "expected turn angle linear");
                return;
            }
            if (s_turnActive)
                turn_finish("SUPERSEDED", NULL, true);
            if (s_moveActive)
                move_finish("SUPERSEDED", NULL);
            const int rc = start_turn(value, linear_m_s, now_us, psi_rad);
            if (rc != TAC_OK) {
                reply_rc("proc", rc);
                return;
            }
            char fields[64];
            const double timeout_s = (double)(s_turnDeadlineUs - now_us) / 1000000.0;
            snprintf(fields, sizeof fields, "name=turn lin=%.3f timeout=%.3f",
                 (double)s_turnLinearMs, timeout_s);
            reply_ok_fields("proc", fields);
        } else {
            if (value == 0.0f || linear_m_s == 0.0f
                    || (value < 0.0f) != (linear_m_s < 0.0f)) {
                reply_err("proc", "bad_args", "distance and linear must agree");
                return;
            }
            if (s_turnActive)
                turn_finish("SUPERSEDED", NULL, true);
            if (s_moveActive)
                move_finish("SUPERSEDED", NULL);
            const int rc = start_move(value, linear_m_s, now_us, psi_rad);
            if (rc != TAC_OK) {
                reply_rc("proc", rc);
                return;
            }
            char fields[64];
            const double timeout_s = (double)(s_moveDeadlineUs - now_us) / 1000000.0;
            snprintf(fields, sizeof fields, "name=move lin=%.3f timeout=%.3f",
                 (double)s_moveLinearMs, timeout_s);
            reply_ok_fields("proc", fields);
        }
    } else if (codec_token_eq(verb, "abort")) {
        turn_finish("ABORTED", "command", true);
        move_finish("ABORTED", "command");
        reply_rc("abort", TAC_OK);
    }
}

bool cockpit_procedure_active(void) { return s_turnActive || s_moveActive; }

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

void cockpit_tick(uint64_t now_us)
{
    if (!s_turnActive && !s_moveActive)
        return;
    if (tac_state() != TacticalState::Active) {
        const char *reason = (tac_state() == TacticalState::Fallback)
            ? "deadman" : "state";
        turn_finish("ABORTED", reason, false);
        move_finish("ABORTED", reason);
        return;
    }

    float psi_rad;
    if (!turn_imu_ready(&psi_rad)) {
        if (s_turnActive)
            turn_finish("ABORTED", "imu_stale", false);
        if (s_moveActive)
            move_finish("ABORTED", "imu_stale");
        return;
    }
    float rate_rad_s, bias_rad_s;
    bool heading_valid;
    s_heading(&psi_rad, &rate_rad_s, &bias_rad_s, &heading_valid);
    (void)bias_rad_s;
    (void)heading_valid;

    if (s_turnActive) {
        s_turnProgressRad += wrap_pi(psi_rad - s_turnLastPsiRad);
        s_turnLastPsiRad = psi_rad;
        const double progress = (s_turnAngleRad > 0.0f)
            ? s_turnProgressRad : -s_turnProgressRad;
        const double target = fmax((double)fabsf(s_turnAngleRad)
                                   - (double)s_turnOvershootRad, 0.0);
        if (progress >= target) {
            turn_finish("DONE", NULL, true);
        } else if (now_us >= s_turnDeadlineUs) {
            turn_finish("ABORTED", "timeout", true);
        }
    }

    if (s_moveActive) {
        int32_t left_ticks, right_ticks;
        float vl, vr;
        s_odom(&left_ticks, &right_ticks, &vl, &vr);
        const double travelled_m = ((double)wrapping_tick_delta(left_ticks, s_moveStartLeft)
                                  + (double)wrapping_tick_delta(right_ticks, s_moveStartRight))
                               / (2.0 * (double)s_ticksPerMeter);
        const double progress_m = (s_moveDistanceM > 0.0f) ? travelled_m : -travelled_m;
        if (progress_m >= fabsf(s_moveDistanceM)) {
            move_finish("DONE", NULL);
        } else if (now_us >= s_moveDeadlineUs) {
            move_finish("ABORTED", "timeout");
        } else {
            const float dt = (float)(now_us - s_moveLastControlUs) * 1e-6f;
            s_moveLastControlUs = now_us;
            const float error = wrap_pi(s_movePsiRefRad - psi_rad);
            if (dt > 0.0f && dt < 0.1f && s_headingKi > 0.0f) {
                const float integral_limit = s_headingIntegralMaxRadS / s_headingKi;
                s_moveHeadingIntegral += error * dt;
                if (s_moveHeadingIntegral > integral_limit)
                    s_moveHeadingIntegral = integral_limit;
                if (s_moveHeadingIntegral < -integral_limit)
                    s_moveHeadingIntegral = -integral_limit;
            }
            float omega = s_headingKp * error
                        + s_headingKi * s_moveHeadingIntegral
                        - s_headingKd * rate_rad_s;
            if (omega > s_headingOmegaMaxRadS)
                omega = s_headingOmegaMaxRadS;
            if (omega < -s_headingOmegaMaxRadS)
                omega = -s_headingOmegaMaxRadS;
            const float left = s_moveLinearMs + omega * s_halfTrackM;
            const float right = s_moveLinearMs - omega * s_halfTrackM;
            const float scale = drive_scale(left, right);
            tac_drive(wheel_mm_s(left * scale), wheel_mm_s(right * scale));
        }
    }
}
