#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Wire layer of the cockpit protocol (protocol/cockpit_protocol.md sections
// 2-4): line assembly, classification, tokenizing, and output formatting.
// Transport-agnostic -- feed it bytes, hand its output to a UART or USB.
// No FSM knowledge, no hardware, unit-tests on the host against the golden
// vectors in protocol/cockpit_vectors.txt.

// Maximum line length including the terminator (spec section 2).
#define CODEC_MAX_LINE   120
#define CODEC_MAX_TOKENS 8

// ---- line assembler: bytes in, complete lines out -------------------------
// One assembler per port (the airframe has a UART cockpit and may add a USB
// debug console), so this is a small struct, not module state.
typedef struct {
    char    buf[CODEC_MAX_LINE];
    uint8_t len;
    bool    overflow;
    bool    complete;
} LineAssembler;

void line_asm_init(LineAssembler *a);

// Feed one received byte. Returns true when a complete line is ready:
// a->buf is NUL-terminated with no CR/LF. Over-length input is discarded to
// the next newline and then reported as a complete line with a->overflow set
// and an empty buffer, so the caller can answer `=err ? line_too_long`.
bool line_asm_feed(LineAssembler *a, char c);

// ---- classification and tokenizing ----------------------------------------
typedef enum {
    LINE_IGNORE,    // blank, `*` comment, or a sigil that is not ours -- skip
    LINE_RELAY,     // leading `^`: *payload points at the verbatim payload
    LINE_REQUEST,   // bare verb [args]
} LineKind;

// Classify an assembled line. For LINE_RELAY, *payload is set (may be "").
LineKind codec_classify(char *line, const char **payload);

// Split a request line into whitespace-separated tokens, in place.
// Returns the token count, or -1 if there are more than max_tokens.
int codec_tokenize(char *line, char *tokens[], int max_tokens);

// Case-insensitive token comparison (spec: verbs are case-insensitive).
bool codec_token_eq(const char *tok, const char *word);

// Whole-token number parsing; trailing garbage or range overflow -> false.
bool codec_parse_i16(const char *tok, int16_t *out);
bool codec_parse_f32(const char *tok, float *out);

// ---- output formatting -----------------------------------------------------
// All formatters write a NUL-terminated line WITHOUT terminator into dst and
// return its length, or -1 if it would not fit dst or the spec line bound.
// The transport appends "\r\n".
int codec_format_ok(char *dst, size_t cap, const char *verb,
                    const char *fields);              // fields NULL/"" = none
int codec_format_err(char *dst, size_t cap, const char *verb,
                     const char *reason, const char *detail);  // detail opt.
int codec_format_state_event(char *dst, size_t cap,
                             const char *from, const char *to);
int codec_format_fault_event(char *dst, size_t cap, const char *code_name);
int codec_format_relay(char *dst, size_t cap, const char *payload);
