#!/usr/bin/env bash
#
# Wanderer Pilot health monitor.
#
# Exit status in --once mode:
#   0 = OK
#   1 = WARN
#   2 = FAULT

set -u
export LC_ALL=C

readonly EXIT_OK=0
readonly EXIT_WARN=1
readonly EXIT_FAULT=2

WIFI_INTERFACE="${WANDERER_WIFI_INTERFACE:-wlan0}"
BASE_HOST="${WANDERER_BASE_HOST:-}"
CHECK_INTERVAL="${WANDERER_CHECK_INTERVAL:-5}"
WEAK_SIGNAL_DBM="${WANDERER_WEAK_SIGNAL_DBM:--68}"
FAULT_SIGNAL_DBM="${WANDERER_FAULT_SIGNAL_DBM:--76}"
RUN_DIR="${WANDERER_RUN_DIR:-/run/wanderer}"
STATE_FILE="${RUN_DIR}/health.env"
LED_ROOT="${WANDERER_LED_ROOT:-/sys/class/leds}"

RED_LED="${LED_ROOT}/wanderer-red/brightness"
GREEN_LED="${LED_ROOT}/wanderer-green/brightness"
BLUE_LED="${LED_ROOT}/wanderer-blue/brightness"

MODE="daemon"
LAST_FINGERPRINT=""

usage() {
    cat <<'EOF'
Usage: wanderer-health.sh [--once] [--no-led] [--help]

  --once    Run one check, print the result, and return 0/1/2.
  --no-led  Do not change the RGB status LED.
  --help    Show this help.

Configuration is read from WANDERER_* environment variables. See health.conf.
EOF
}

LED_ENABLED=1
while (($# > 0)); do
    case "$1" in
        --once)
            MODE="once"
            ;;
        --no-led)
            LED_ENABLED=0
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            printf 'Unknown argument: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

is_integer() {
    [[ "$1" =~ ^-?[0-9]+$ ]]
}

if ! is_integer "$CHECK_INTERVAL" || ((CHECK_INTERVAL < 1)); then
    printf 'WANDERER_CHECK_INTERVAL must be a positive integer\n' >&2
    exit 2
fi
if ! is_integer "$WEAK_SIGNAL_DBM" || ! is_integer "$FAULT_SIGNAL_DBM"; then
    printf 'Wi-Fi signal thresholds must be integers\n' >&2
    exit 2
fi
if ((FAULT_SIGNAL_DBM >= WEAK_SIGNAL_DBM)); then
    printf 'WANDERER_FAULT_SIGNAL_DBM must be lower than WANDERER_WEAK_SIGNAL_DBM\n' >&2
    exit 2
fi

command_exists() {
    command -v "$1" >/dev/null 2>&1
}

read_first_line() {
    local value
    IFS= read -r value || true
    printf '%s' "$value"
}

set_led_channels() {
    local red="$1"
    local green="$2"
    local blue="$3"

    ((LED_ENABLED == 1)) || return 0
    [[ -w "$RED_LED" && -w "$GREEN_LED" && -w "$BLUE_LED" ]] || return 0

    printf '%s\n' "$red" >"$RED_LED"
    printf '%s\n' "$green" >"$GREEN_LED"
    printf '%s\n' "$blue" >"$BLUE_LED"
}

set_led_state() {
    case "$1" in
        ok)      set_led_channels 0 1 0 ;; # green
        warn)    set_led_channels 1 1 0 ;; # yellow
        fault)   set_led_channels 1 0 0 ;; # red
        power)   set_led_channels 1 0 1 ;; # magenta
        startup) set_led_channels 0 0 1 ;; # blue
        off)     set_led_channels 0 0 0 ;;
    esac
}

cleanup() {
    set_led_state off
}

wifi_radio_state() {
    local state

    if command_exists rfkill; then
        state="$(rfkill -n -o SOFT,HARD list wifi 2>/dev/null | head -n 1 || true)"
        if awk '{ exit !($1 == "blocked" || $2 == "blocked") }' <<<"$state"; then
            printf 'off'
            return
        fi
        if [[ -n "$state" ]]; then
            printf 'on'
            return
        fi
    fi

    if [[ -e "/sys/class/net/${WIFI_INTERFACE}" ]]; then
        printf 'on'
    else
        printf 'unavailable'
    fi
}

wifi_link_data() {
    if command_exists iw; then
        iw dev "$WIFI_INTERFACE" link 2>/dev/null || true
    fi
}

wifi_ssid_from_link() {
    local link_data="$1"
    sed -n 's/^[[:space:]]*SSID: //p' <<<"$link_data" | head -n 1
}

wifi_signal_from_link() {
    local link_data="$1"
    sed -n 's/^[[:space:]]*signal: \(-\{0,1\}[0-9]\+\).*/\1/p' \
        <<<"$link_data" | head -n 1
}

wifi_associated_state() {
    local link_data="$1"

    if [[ "$link_data" == *"Connected to "* ]]; then
        printf 'yes'
    elif command_exists nmcli &&
         [[ "$(nmcli -g GENERAL.STATE device show "$WIFI_INTERFACE" 2>/dev/null || true)" == 100* ]]; then
        printf 'yes'
    else
        printf 'no'
    fi
}

wifi_ssid_fallback() {
    if command_exists nmcli; then
        nmcli -g GENERAL.CONNECTION device show "$WIFI_INTERFACE" 2>/dev/null |
            read_first_line
    fi
}

ipv4_address() {
    if command_exists ip; then
        ip -4 -o address show dev "$WIFI_INTERFACE" scope global 2>/dev/null |
            awk 'NR == 1 { sub(/\/.*/, "", $4); print $4 }'
    fi
}

default_route_state() {
    if command_exists ip &&
       ip -4 route show default dev "$WIFI_INTERFACE" 2>/dev/null | grep -q .; then
        printf 'yes'
    else
        printf 'no'
    fi
}

base_reachable_state() {
    if [[ -z "$BASE_HOST" ]]; then
        printf 'not_configured'
    elif command_exists ping && ping -n -c 1 -W 1 "$BASE_HOST" >/dev/null 2>&1; then
        printf 'yes'
    else
        printf 'no'
    fi
}

signal_quality() {
    local signal="$1"

    if ! is_integer "$signal"; then
        printf 'unknown'
    elif ((signal >= -55)); then
        printf 'excellent'
    elif ((signal >= WEAK_SIGNAL_DBM)); then
        printf 'good'
    elif ((signal >= FAULT_SIGNAL_DBM)); then
        printf 'usable'
    else
        printf 'weak'
    fi
}

power_status() {
    local raw

    THROTTLED_HEX="unavailable"
    UNDERVOLTAGE_NOW="unknown"
    UNDERVOLTAGE_HISTORY="unknown"
    THROTTLED_NOW="unknown"
    THROTTLED_HISTORY="unknown"

    command_exists vcgencmd || return 0

    raw="$(vcgencmd get_throttled 2>/dev/null || true)"
    [[ "$raw" =~ 0x([[:xdigit:]]+) ]] || return 0

    THROTTLED_HEX="0x${BASH_REMATCH[1]}"
    local value=$((16#${BASH_REMATCH[1]}))

    ((value & 0x1))     && UNDERVOLTAGE_NOW="yes"     || UNDERVOLTAGE_NOW="no"
    ((value & 0x10000)) && UNDERVOLTAGE_HISTORY="yes" || UNDERVOLTAGE_HISTORY="no"
    ((value & 0x4))     && THROTTLED_NOW="yes"        || THROTTLED_NOW="no"
    ((value & 0x40000)) && THROTTLED_HISTORY="yes"    || THROTTLED_HISTORY="no"
}

shell_quote() {
    printf '%q' "$1"
}

emit_state_lines() {
    printf 'overall=%s\n' "$OVERALL"
    printf 'reason=%s\n' "$(shell_quote "$REASON")"
    printf 'timestamp=%s\n' "$(date --iso-8601=seconds)"
    printf 'wifi_interface=%s\n' "$(shell_quote "$WIFI_INTERFACE")"
    printf 'wifi_radio=%s\n' "$WIFI_RADIO"
    printf 'wifi_associated=%s\n' "$WIFI_ASSOCIATED"
    printf 'wifi_ssid=%s\n' "$(shell_quote "$WIFI_SSID")"
    printf 'wifi_signal_dbm=%s\n' "$WIFI_SIGNAL_DBM"
    printf 'wifi_quality=%s\n' "$WIFI_QUALITY"
    printf 'ipv4=%s\n' "$(shell_quote "$IPV4")"
    printf 'default_route=%s\n' "$DEFAULT_ROUTE"
    printf 'base_host=%s\n' "$(shell_quote "$BASE_HOST")"
    printf 'base_reachable=%s\n' "$BASE_REACHABLE"
    printf 'throttled_hex=%s\n' "$THROTTLED_HEX"
    printf 'undervoltage_now=%s\n' "$UNDERVOLTAGE_NOW"
    printf 'undervoltage_history=%s\n' "$UNDERVOLTAGE_HISTORY"
    printf 'throttled_now=%s\n' "$THROTTLED_NOW"
    printf 'throttled_history=%s\n' "$THROTTLED_HISTORY"
}

emit_state() {
    local destination="${1:--}"

    if [[ "$destination" == "-" ]]; then
        emit_state_lines
    else
        emit_state_lines >"$destination"
    fi
}

write_state_file() {
    local temporary_file

    mkdir -p "$RUN_DIR"
    temporary_file="${STATE_FILE}.tmp.$$"
    emit_state "$temporary_file"
    chmod 0644 "$temporary_file"
    mv -f "$temporary_file" "$STATE_FILE"
}

log_state_change() {
    local fingerprint
    local summary="${OVERALL}: ${REASON}"

    fingerprint="${OVERALL}|${WIFI_RADIO}|${WIFI_ASSOCIATED}|${WIFI_QUALITY}"
    fingerprint+="|${IPV4/none/}|${DEFAULT_ROUTE}|${BASE_REACHABLE}"
    fingerprint+="|${UNDERVOLTAGE_NOW}|${UNDERVOLTAGE_HISTORY}"
    fingerprint+="|${THROTTLED_NOW}|${THROTTLED_HISTORY}"

    if [[ "$fingerprint" != "$LAST_FINGERPRINT" ]]; then
        printf 'Wanderer health changed: %s\n' "$summary"
        LAST_FINGERPRINT="$fingerprint"
    fi
}

perform_check() {
    local link_data
    local -a fault_reasons=()
    local -a warn_reasons=()

    WIFI_RADIO="$(wifi_radio_state)"
    link_data="$(wifi_link_data)"
    WIFI_ASSOCIATED="$(wifi_associated_state "$link_data")"
    WIFI_SSID="$(wifi_ssid_from_link "$link_data")"
    [[ -n "$WIFI_SSID" ]] || WIFI_SSID="$(wifi_ssid_fallback)"
    WIFI_SIGNAL_DBM="$(wifi_signal_from_link "$link_data")"
    [[ -n "$WIFI_SIGNAL_DBM" ]] || WIFI_SIGNAL_DBM="unknown"
    WIFI_QUALITY="$(signal_quality "$WIFI_SIGNAL_DBM")"
    IPV4="$(ipv4_address)"
    [[ -n "$IPV4" ]] || IPV4="none"
    DEFAULT_ROUTE="$(default_route_state)"
    BASE_REACHABLE="$(base_reachable_state)"
    power_status

    [[ "$WIFI_RADIO" == "on" ]] ||
        fault_reasons+=("Wi-Fi radio ${WIFI_RADIO}")
    [[ "$WIFI_ASSOCIATED" == "yes" ]] ||
        fault_reasons+=("Wi-Fi not associated")
    [[ "$IPV4" != "none" ]] ||
        fault_reasons+=("no IPv4 address")

    [[ "$DEFAULT_ROUTE" == "yes" ]] ||
        warn_reasons+=("no Wi-Fi default route")
    [[ "$BASE_REACHABLE" != "no" ]] ||
        warn_reasons+=("Base ${BASE_HOST} unreachable")

    if is_integer "$WIFI_SIGNAL_DBM"; then
        if ((WIFI_SIGNAL_DBM < FAULT_SIGNAL_DBM)); then
            fault_reasons+=("Wi-Fi signal ${WIFI_SIGNAL_DBM} dBm")
        elif ((WIFI_SIGNAL_DBM < WEAK_SIGNAL_DBM)); then
            warn_reasons+=("Wi-Fi signal ${WIFI_SIGNAL_DBM} dBm")
        fi
    else
        warn_reasons+=("Wi-Fi signal unavailable")
    fi

    [[ "$UNDERVOLTAGE_NOW" != "yes" ]] ||
        fault_reasons+=("undervoltage active")
    [[ "$THROTTLED_NOW" != "yes" ]] ||
        fault_reasons+=("CPU throttling active")
    [[ "$UNDERVOLTAGE_HISTORY" != "yes" ]] ||
        warn_reasons+=("undervoltage occurred since boot")
    [[ "$THROTTLED_HISTORY" != "yes" ]] ||
        warn_reasons+=("CPU throttling occurred since boot")

    if ((${#fault_reasons[@]} > 0)); then
        OVERALL="FAULT"
        REASON="$(IFS='; '; printf '%s' "${fault_reasons[*]}")"
        if [[ "$UNDERVOLTAGE_NOW" == "yes" || "$THROTTLED_NOW" == "yes" ]]; then
            set_led_state power
        else
            set_led_state fault
        fi
        return "$EXIT_FAULT"
    fi

    if ((${#warn_reasons[@]} > 0)); then
        OVERALL="WARN"
        REASON="$(IFS='; '; printf '%s' "${warn_reasons[*]}")"
        set_led_state warn
        return "$EXIT_WARN"
    fi

    OVERALL="OK"
    REASON="Wi-Fi and power checks passed"
    set_led_state ok
    return "$EXIT_OK"
}

run_once() {
    local status

    perform_check
    status=$?
    write_state_file
    emit_state -
    return "$status"
}

run_daemon() {
    set_led_state startup
    sleep 1

    while true; do
        perform_check || true
        write_state_file
        log_state_change
        sleep "$CHECK_INTERVAL"
    done
}

if [[ "$MODE" == "once" ]]; then
    run_once
else
    trap cleanup EXIT
    trap 'exit 0' INT TERM
    run_daemon
fi
