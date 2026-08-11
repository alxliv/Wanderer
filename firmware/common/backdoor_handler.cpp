#include "backdoor_handler.h"

#include <stdio.h>
#include <string.h>

#include "cockpit_codec.h"
#include "tactical.h"

// ---- module state ----------------------------------------------------------

static backdoor_sink         s_sink;
static backdoor_motor_fn     s_motor;
static backdoor_enc_fn       s_enc;
static backdoor_enc_reset_fn s_encReset;
static uint8_t               s_fwMajor, s_fwMinor;
static LineAssembler         s_asm;

// Outstanding wiggle. s_wiggleUntilUs is meaningless unless s_wiggling.
static bool     s_wiggling;
static uint64_t s_wiggleUntilUs;

static void emit(const char *line)
{
    if (s_sink)
        s_sink(line);
}

static void motors_zero(void)
{
    if (s_motor)
        s_motor(0, 0);
}

// Every path that ends a wiggle goes through here, so there is exactly one
// place where the wheels are commanded to stop and one place where the flag
// is cleared. `announce` is false when the caller is already replying.
static void end_wiggle(bool announce, const char *why)
{
    if (!s_wiggling)
        return;
    s_wiggling = false;
    motors_zero();
    if (announce) {
        char line[CODEC_MAX_LINE];
        snprintf(line, sizeof line, "!wiggle_done %s", why);
        emit(line);
    }
}

void backdoor_init(backdoor_sink sink, uint8_t fw_major, uint8_t fw_minor)
{
    s_sink = sink;
    s_fwMajor = fw_major;
    s_fwMinor = fw_minor;
    s_motor = NULL;
    s_enc = NULL;
    s_encReset = NULL;
    s_wiggling = false;
    s_wiggleUntilUs = 0;
    line_asm_init(&s_asm);
}

void backdoor_set_motor_sink(backdoor_motor_fn fn) { s_motor = fn; }

void backdoor_set_encoder_provider(backdoor_enc_fn fn, backdoor_enc_reset_fn reset)
{
    s_enc = fn;
    s_encReset = reset;
}

bool backdoor_wiggle_active(void) { return s_wiggling; }

void backdoor_abort(void)
{
    end_wiggle(false, NULL);
    tac_dev_release();
    line_asm_init(&s_asm);
}

// ---- replies ---------------------------------------------------------------

static void reply_ok(const char *verb, const char *fields)
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

static void reply_rc(const char *verb, int rc)
{
    if (rc == TAC_OK)
        reply_ok(verb, NULL);
    else
        reply_err(verb, tac_strerror(rc), NULL);
}

// ---- verbs -----------------------------------------------------------------

static int16_t clamp_duty(int16_t v)
{
    if (v >  BACKDOOR_MAX_DUTY_PERMILLE) return  BACKDOOR_MAX_DUTY_PERMILLE;
    if (v < -BACKDOOR_MAX_DUTY_PERMILLE) return -BACKDOOR_MAX_DUTY_PERMILLE;
    return v;
}

static void do_dev(char *tokens[], int n, uint64_t now_us)
{
    if (n != 2) {
        reply_err("dev", "bad_args", "on|off");
        return;
    }
    if (codec_token_eq(tokens[1], "on")) {
        const int rc = tac_dev_acquire(now_us);
        reply_rc("dev", rc);
        return;
    }
    if (codec_token_eq(tokens[1], "off")) {
        end_wiggle(false, NULL);
        tac_dev_release();
        reply_ok("dev", NULL);
        return;
    }
    reply_err("dev", "bad_args", "on|off");
}

static void do_wiggle(char *tokens[], int n, uint64_t now_us)
{
    if (!tac_dev_active()) {
        // The gate. Refusing here is the whole reason dev mode exists: the
        // Base must not be able to move wheels the Pilot believes it owns.
        reply_err("wiggle", tac_strerror(TAC_ERR_DEV_INACTIVE), "dev on first");
        return;
    }
    if (n != 4) {
        reply_err("wiggle", "bad_args", "<left> <right> <ms>");
        return;
    }
    // Duration is parsed as float, not i16, purely so that an out-of-range
    // duration reports as an over-long duration (and clamps) instead of as
    // "not_a_number": 60000 ms is a plausible typo, and i16 would reject it
    // with an error that sends the operator hunting for a syntax problem.
    int16_t l, r;
    float   ms_f;
    if (!codec_parse_i16(tokens[1], &l) ||
        !codec_parse_i16(tokens[2], &r) ||
        !codec_parse_f32(tokens[3], &ms_f)) {
        reply_err("wiggle", "bad_args", "not_a_number");
        return;
    }
    if (ms_f < 1.0f) {
        reply_err("wiggle", "bad_args", "ms_must_be_positive");
        return;
    }
    int32_t ms = (ms_f > (float)BACKDOOR_MAX_WIGGLE_MS)
                     ? BACKDOOR_MAX_WIGGLE_MS
                     : (int32_t)ms_f;

    // Clamp rather than refuse: a sweep script walking duty upward should not
    // have to know the ceiling, and the reply reports what was actually
    // applied so the operator's log is truthful.
    l = clamp_duty(l);
    r = clamp_duty(r);

    s_wiggling = true;
    s_wiggleUntilUs = now_us + (uint64_t)ms * 1000ull;
    if (s_motor)
        s_motor(l, r);

    char fields[64];
    snprintf(fields, sizeof fields, "l=%d r=%d ms=%d", (int)l, (int)r, (int)ms);
    reply_ok("wiggle", fields);
}

static void do_enc(char *tokens[], int n)
{
    if (n == 2 && codec_token_eq(tokens[1], "reset")) {
        if (s_encReset)
            s_encReset();
        reply_ok("enc", "left=0 right=0");
        return;
    }
    if (n != 1) {
        reply_err("enc", "bad_args", "enc|enc reset");
        return;
    }
    int32_t lt = 0, rt = 0;
    if (s_enc)
        s_enc(&lt, &rt);
    char fields[64];
    snprintf(fields, sizeof fields, "left=%ld right=%ld", (long)lt, (long)rt);
    reply_ok("enc", fields);
}

static void do_ver(void)
{
    char fields[80];
    snprintf(fields, sizeof fields,
             "fw=%u.%u iface=backdoor max_duty=%d max_ms=%d",
             (unsigned)s_fwMajor, (unsigned)s_fwMinor,
             (int)BACKDOOR_MAX_DUTY_PERMILLE, (int)BACKDOOR_MAX_WIGGLE_MS);
    reply_ok("ver", fields);
}

static void do_help(void)
{
    emit("*backdoor verbs: dev on|off, wiggle <l> <r> <ms>, enc [reset],");
    emit("*  estop, safe, clear_fault, ver, help");
    emit("*wiggle needs dev on, which needs SAFE + no live cockpit commander");
    emit("*estop latches FAULT; clear_fault lifts it, then safe/dev on works again");
    reply_ok("help", NULL);
}

// ---- request dispatch ------------------------------------------------------

static void dispatch(char *line, uint64_t now_us)
{
    char *tokens[CODEC_MAX_TOKENS];
    const int n = codec_tokenize(line, tokens, CODEC_MAX_TOKENS);
    if (n <= 0) {
        reply_err("?", "malformed", NULL);
        return;
    }
    const char *verb = tokens[0];

    // ESTOP first and unconditionally: honored from any source, always,
    // regardless of authority or mode (architecture section 3a). It is the
    // one verb that must never consult the gate.
    if (codec_token_eq(verb, "estop")) {
        end_wiggle(false, NULL);
        tac_dev_release();
        reply_rc("estop", tac_estop());
    } else if (codec_token_eq(verb, "safe")) {
        end_wiggle(false, NULL);
        tac_dev_release();
        reply_rc("safe", tac_disarm());
    } else if (codec_token_eq(verb, "clear_fault")) {
        // Same call the cockpit's clear_fault makes: ESTOP's condition is
        // definitionally gone once commanded away (spec section 6). Tier 3
        // faults will plug a real check in here on both interfaces at once.
        reply_rc("clear_fault", tac_clear_fault(true));
    } else if (codec_token_eq(verb, "dev")) {
        do_dev(tokens, n, now_us);
    } else if (codec_token_eq(verb, "wiggle")) {
        do_wiggle(tokens, n, now_us);
    } else if (codec_token_eq(verb, "enc")) {
        do_enc(tokens, n);
    } else if (codec_token_eq(verb, "ver")) {
        do_ver();
    } else if (codec_token_eq(verb, "help")) {
        do_help();
    } else {
        reply_err(verb, "unknown_command", NULL);
    }
}

void backdoor_feed(char c, uint64_t now_us)
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
        dispatch(s_asm.buf, now_us);
        break;
    case LINE_RELAY:   // the backdoor is not a relay; drop it explicitly
    case LINE_IGNORE:
    default:
        break;
    }
}

void backdoor_tick(uint64_t now_us)
{
    if (!s_wiggling)
        return;

    // Losing the lease mid-wiggle means the cockpit woke up, a fault latched,
    // or someone hit ESTOP. Any of those outrank the bench, so stop now and
    // say why -- a silently truncated sweep step would corrupt the operator's
    // deadband reading without them noticing.
    if (!tac_dev_active()) {
        end_wiggle(true, "lease_lost");
        return;
    }
    if (now_us >= s_wiggleUntilUs)
        end_wiggle(true, "timeout");
}
