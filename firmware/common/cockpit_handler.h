#pragma once
#include <stdint.h>

// The airframe's cockpit endpoint (protocol/cockpit_protocol.md): receives
// request lines, drives the tactical FSM, emits reply and event lines.
// Transport-agnostic -- the caller pumps received bytes in and provides a
// sink for outgoing lines; on the airframe both ends are a UART, in host
// tests they are strings. One cockpit per vehicle: a module, like tactical.
//
// Division of labor: cockpit_codec is the wire syntax, tactical is the FSM,
// this module is the dispatch between them plus the spec's liveness rule
// (a well-formed request with a KNOWN verb refreshes the motion lease, even
// when refused; unknown verbs, malformed, relay and ignored lines do not).

// Receives one outgoing line, NUL-terminated, WITHOUT terminator; the
// transport appends "\r\n".
typedef void (*cockpit_sink)(const char *line);

// Optional odometry source for get_odometry; NULL reports zeros.
typedef void (*cockpit_odom_fn)(int32_t *left_ticks, int32_t *right_ticks,
                                float *left_m_s, float *right_m_s);

// Wire up sink, version, and vehicle geometry (used to convert the drive
// request's body velocities into wheel targets). Registers the tactical
// change-state callback: every FSM transition emits `!state` (preceded by
// `!fault` when latching), whatever caused it.
void cockpit_init(cockpit_sink sink, uint8_t fw_major, uint8_t fw_minor,
                  float half_track_m);

void cockpit_set_odometry_provider(cockpit_odom_fn fn);

// Future radio-modem hat: `^` payloads are handed here verbatim (spec
// section 4). NULL (the default) drops them. Never refreshes the lease.
void cockpit_set_relay_sink(cockpit_sink fn);

// Pump one received byte. Dispatches when a full line has arrived.
void cockpit_feed(char c, uint64_t now_us);
