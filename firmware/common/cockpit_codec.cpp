#include "cockpit_codec.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---- line assembler --------------------------------------------------------

void line_asm_init(LineAssembler *a)
{
    a->len = 0;
    a->overflow = false;
    a->complete = false;
    a->buf[0] = '\0';
}

bool line_asm_feed(LineAssembler *a, char c)
{
    if (a->complete)              // previous line was consumed; start fresh
        line_asm_init(a);

    if (c == '\r')                // tolerated, never stored (spec section 2)
        return false;

    if (c == '\n') {
        if (a->overflow)
            a->buf[0] = '\0';     // report the overflow with an empty line
        else
            a->buf[a->len] = '\0';
        a->complete = true;
        return true;
    }

    if (a->overflow)              // discarding to the next newline
        return false;

    if (a->len >= CODEC_MAX_LINE - 1) {
        a->overflow = true;       // line bound exceeded; keep discarding
        return false;
    }

    a->buf[a->len++] = c;
    return false;
}

// ---- classification and tokenizing ----------------------------------------

LineKind codec_classify(char *line, const char **payload)
{
    while (*line == ' ' || *line == '\t')
        ++line;
    if (*line == '^') {
        if (payload)
            *payload = line + 1;  // verbatim, untrimmed
        return LINE_RELAY;
    }
    if (isalpha((unsigned char)*line))
        return LINE_REQUEST;
    return LINE_IGNORE;           // blank, `*`, or a sigil that is not ours
}

int codec_tokenize(char *line, char *tokens[], int max_tokens)
{
    int count = 0;
    char *p = line;
    while (*p) {
        while (*p == ' ' || *p == '\t')
            ++p;
        if (!*p)
            break;
        if (count == max_tokens)
            return -1;            // too many tokens: malformed
        tokens[count++] = p;
        while (*p && *p != ' ' && *p != '\t')
            ++p;
        if (*p)
            *p++ = '\0';
    }
    return count;
}

bool codec_token_eq(const char *tok, const char *word)
{
    while (*tok && *word) {
        if (tolower((unsigned char)*tok) != tolower((unsigned char)*word))
            return false;
        ++tok;
        ++word;
    }
    return *tok == '\0' && *word == '\0';
}

bool codec_parse_i16(const char *tok, int16_t *out)
{
    if (*tok == '\0')
        return false;
    char *end = NULL;
    long v = strtol(tok, &end, 10);
    if (*end != '\0' || v < INT16_MIN || v > INT16_MAX)
        return false;
    *out = (int16_t)v;
    return true;
}

bool codec_parse_f32(const char *tok, float *out)
{
    if (*tok == '\0')
        return false;
    char *end = NULL;
    float v = strtof(tok, &end);
    if (*end != '\0' || !isfinite(v))
        return false;
    *out = v;
    return true;
}

// ---- output formatting -----------------------------------------------------

static int finish(char *dst, size_t cap, int n)
{
    if (n < 0 || (size_t)n >= cap || n > CODEC_MAX_LINE - 2)
        return -1;                // -2: leave room for the "\r\n" terminator
    (void)dst;
    return n;
}

int codec_format_ok(char *dst, size_t cap, const char *verb,
                    const char *fields)
{
    int n;
    if (fields && fields[0])
        n = snprintf(dst, cap, "=ok %s %s", verb, fields);
    else
        n = snprintf(dst, cap, "=ok %s", verb);
    return finish(dst, cap, n);
}

int codec_format_err(char *dst, size_t cap, const char *verb,
                     const char *reason, const char *detail)
{
    int n;
    if (detail && detail[0])
        n = snprintf(dst, cap, "=err %s %s %s", verb, reason, detail);
    else
        n = snprintf(dst, cap, "=err %s %s", verb, reason);
    return finish(dst, cap, n);
}

int codec_format_state_event(char *dst, size_t cap,
                             const char *from, const char *to)
{
    return finish(dst, cap, snprintf(dst, cap, "!state from=%s to=%s", from, to));
}

int codec_format_fault_event(char *dst, size_t cap, const char *code_name)
{
    return finish(dst, cap, snprintf(dst, cap, "!fault code=%s", code_name));
}

int codec_format_relay(char *dst, size_t cap, const char *payload)
{
    return finish(dst, cap, snprintf(dst, cap, "^%s", payload));
}
