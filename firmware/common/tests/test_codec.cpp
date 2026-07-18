// Host-side tests for the cockpit line codec, driven by the golden vectors
// in protocol/cockpit_vectors.txt (plus a few byte-level assembler cases the
// vector format cannot express).
// Build (cmake): see CMakeLists.txt in this directory.
// Build (plain): g++ -std=c++17 -Wall -Wextra -I.. test_codec.cpp \
//                    ../cockpit_codec.cpp -o test_codec
// Run: ./test_codec [path/to/cockpit_vectors.txt]

#define _CRT_SECURE_NO_WARNINGS  // MSVC: fopen is fine for a test reading vectors

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "cockpit_codec.h"

#ifndef VECTORS_PATH
#define VECTORS_PATH "../../protocol/cockpit_vectors.txt"
#endif

static int failures = 0;
static int vectors_run = 0;
#define CHECK(cond, msg, ctx) do { \
    if (!(cond)) { printf("FAIL line %d: %s  [%s]\n", __LINE__, msg, ctx); failures++; } \
} while (0)

// Split a vector line on '|' in place. Returns field count.
// An empty trailing field IS a field ("a|b|" -> 3).
static int split(char *line, char *fields[], int max)
{
    int n = 0;
    fields[n++] = line;
    for (char *p = line; *p && n < max; ++p) {
        if (*p == '|') {
            *p = '\0';
            fields[n++] = p + 1;
        }
    }
    return n;
}

// ---- [parse] ---------------------------------------------------------------

static void run_parse_vector(char *fields[], int n, const char *ctx)
{
    char work[CODEC_MAX_LINE];
    snprintf(work, sizeof work, "%s", fields[0]);
    const char *payload = NULL;
    LineKind kind = codec_classify(work, &payload);

    if (strcmp(fields[1], "ignore") == 0) {
        CHECK(kind == LINE_IGNORE, "expected ignore", ctx);
        return;
    }
    if (strcmp(fields[1], "relay") == 0) {
        CHECK(kind == LINE_RELAY, "expected relay", ctx);
        CHECK(kind == LINE_RELAY && strcmp(payload, fields[2]) == 0,
              "relay payload verbatim", ctx);
        return;
    }
    CHECK(kind == LINE_REQUEST, "expected request", ctx);
    if (kind != LINE_REQUEST)
        return;
    char *tok[CODEC_MAX_TOKENS];
    int count = codec_tokenize(work, tok, CODEC_MAX_TOKENS);
    CHECK(count == n - 2, "token count", ctx);
    for (int i = 0; i < count && i < n - 2; ++i)
        CHECK(strcmp(tok[i], fields[2 + i]) == 0, "token text", ctx);
}

// ---- [format] --------------------------------------------------------------

static void run_format_vector(char *fields[], int n, const char *ctx)
{
    char out[CODEC_MAX_LINE];
    int len = -1;
    const char *expected = fields[n - 1];

    if (strcmp(fields[0], "ok") == 0 && n == 4)
        len = codec_format_ok(out, sizeof out, fields[1], fields[2]);
    else if (strcmp(fields[0], "err") == 0 && n == 5)
        len = codec_format_err(out, sizeof out, fields[1], fields[2], fields[3]);
    else if (strcmp(fields[0], "state") == 0 && n == 4)
        len = codec_format_state_event(out, sizeof out, fields[1], fields[2]);
    else if (strcmp(fields[0], "fault") == 0 && n == 3)
        len = codec_format_fault_event(out, sizeof out, fields[1]);
    else {
        CHECK(false, "malformed format vector", ctx);
        return;
    }
    CHECK(len == (int)strlen(expected), "formatted length", ctx);
    CHECK(len >= 0 && strcmp(out, expected) == 0, "formatted text", ctx);
}

// ---- [request] -------------------------------------------------------------
// The pilot formats these; the airframe must parse them back to the same
// values. Round-trip check: tokenize the expected wire line and re-parse.

static void run_request_vector(char *fields[], int n, const char *ctx)
{
    char work[CODEC_MAX_LINE];
    snprintf(work, sizeof work, "%s", fields[n - 1]);
    const char *payload = NULL;
    CHECK(codec_classify(work, &payload) == LINE_REQUEST,
          "request line classifies as request", ctx);
    char *tok[CODEC_MAX_TOKENS];
    int count = codec_tokenize(work, tok, CODEC_MAX_TOKENS);

    if (strcmp(fields[0], "drive") == 0 && n == 4) {
        CHECK(count == 3, "drive has 2 args", ctx);
        if (count != 3)
            return;
        CHECK(codec_token_eq(tok[0], "drive"), "verb", ctx);
        float lin = 0, ang = 0, want_lin = 0, want_ang = 0;
        CHECK(codec_parse_f32(tok[1], &lin), "linear parses", ctx);
        CHECK(codec_parse_f32(tok[2], &ang), "angular parses", ctx);
        want_lin = strtof(fields[1], NULL);
        want_ang = strtof(fields[2], NULL);
        CHECK(fabsf(lin - want_lin) < 1e-3f, "linear round-trips", ctx);
        CHECK(fabsf(ang - want_ang) < 1e-3f, "angular round-trips", ctx);
        return;
    }
    // Bare verbs: one token, equal (case-insensitively) to the op name.
    CHECK(count == 1, "bare verb is one token", ctx);
    CHECK(count == 1 && codec_token_eq(tok[0], fields[0]), "verb matches op", ctx);
}

// ---- vector file driver ----------------------------------------------------

static void run_vectors(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        printf("FAIL: cannot open vectors file: %s\n", path);
        failures++;
        return;
    }
    char line[512];
    char section[32] = "";
    while (fgets(line, sizeof line, f)) {
        line[strcspn(line, "\r\n")] = '\0';
        if (line[0] == '#')
            continue;
        if (line[0] == '[') {
            snprintf(section, sizeof section, "%.30s", line);
            continue;
        }
        if (line[0] == '\0')
            continue;

        char ctx[128];
        snprintf(ctx, sizeof ctx, "%s %.90s", section, line);
        char *fields[CODEC_MAX_TOKENS + 2];
        int n = split(line, fields, CODEC_MAX_TOKENS + 2);

        if (strcmp(section, "[parse]") == 0)
            run_parse_vector(fields, n, ctx);
        else if (strcmp(section, "[format]") == 0)
            run_format_vector(fields, n, ctx);
        else if (strcmp(section, "[request]") == 0)
            run_request_vector(fields, n, ctx);
        else if (strcmp(section, "[downlink]") == 0)
            continue;   // pilot-side direction, tested in Python
        else {
            CHECK(false, "unknown section", ctx);
            continue;
        }
        vectors_run++;
    }
    fclose(f);
}

// ---- assembler: byte-level behavior the vectors cannot express -------------

static void run_assembler_tests(void)
{
    LineAssembler a;
    line_asm_init(&a);

    // CRLF line, then an LF-only line, through one assembler.
    const char *bytes = "arm\r\ndrive 1 2\n";
    int lines = 0;
    for (const char *p = bytes; *p; ++p) {
        if (line_asm_feed(&a, *p)) {
            ++lines;
            if (lines == 1)
                CHECK(strcmp(a.buf, "arm") == 0, "CRLF stripped", "asm");
            if (lines == 2)
                CHECK(strcmp(a.buf, "drive 1 2") == 0, "LF line", "asm");
            CHECK(!a.overflow, "no overflow", "asm");
        }
    }
    CHECK(lines == 2, "two lines assembled", "asm");

    // Over-length input: discarded to newline, reported once as overflow,
    // and the next line is clean.
    line_asm_init(&a);
    for (int i = 0; i < 300; ++i)
        CHECK(!line_asm_feed(&a, 'x'), "no line during overflow", "asm");
    CHECK(line_asm_feed(&a, '\n'), "overflow line completes", "asm");
    CHECK(a.overflow && a.buf[0] == '\0', "overflow flagged, buffer empty", "asm");
    const char *next = "ping\n";
    for (const char *p = next; *p; ++p) {
        if (line_asm_feed(&a, *p)) {
            CHECK(!a.overflow && strcmp(a.buf, "ping") == 0,
                  "clean line after overflow", "asm");
        }
    }

    // Formatter refuses output that breaks the line bound.
    char big[CODEC_MAX_LINE + 32];
    memset(big, 'y', sizeof big - 1);
    big[sizeof big - 1] = '\0';
    char out[sizeof big + 16];
    CHECK(codec_format_ok(out, sizeof out, "get_map", big) == -1,
          "over-bound output refused", "fmt");
}

int main(int argc, char **argv)
{
    run_vectors(argc > 1 ? argv[1] : VECTORS_PATH);
    run_assembler_tests();
    if (failures == 0)
        printf("OK: %d vectors + assembler tests pass\n", vectors_run);
    else
        printf("%d FAILURES\n", failures);
    return failures == 0 ? 0 : 1;
}
