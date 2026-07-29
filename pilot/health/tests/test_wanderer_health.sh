#!/usr/bin/env bash

set -u

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HEALTH_SCRIPT="${SCRIPT_DIR}/../wanderer-health.sh"
TEST_ROOT="$(mktemp -d)"
MOCK_BIN="${TEST_ROOT}/bin"
LED_ROOT="${TEST_ROOT}/leds"
RUN_DIR="${TEST_ROOT}/run"

cleanup() {
    rm -rf -- "$TEST_ROOT"
}
trap cleanup EXIT

mkdir -p "$MOCK_BIN" "$RUN_DIR"
for color in red green blue; do
    mkdir -p "${LED_ROOT}/wanderer-${color}"
    : >"${LED_ROOT}/wanderer-${color}/brightness"
done

cat >"${MOCK_BIN}/rfkill" <<'EOF'
#!/usr/bin/env bash
printf 'unblocked unblocked\n'
EOF

cat >"${MOCK_BIN}/iw" <<'EOF'
#!/usr/bin/env bash
if [[ "${MOCK_ASSOCIATED:-yes}" == "yes" ]]; then
    cat <<DATA
Connected to 00:11:22:33:44:55 (on wlan0)
	SSID: WandererTest
	signal: ${MOCK_SIGNAL_DBM:--60} dBm
DATA
else
    printf 'Not connected.\n'
fi
EOF

cat >"${MOCK_BIN}/ip" <<'EOF'
#!/usr/bin/env bash
if [[ "$*" == "-4 -o address show dev wlan0 scope global" ]]; then
    if [[ "${MOCK_HAS_IPV4:-yes}" == "yes" ]]; then
        printf '3: wlan0    inet 192.168.50.4/24 brd 192.168.50.255 scope global wlan0\n'
    fi
elif [[ "$*" == "-4 route show default dev wlan0" ]]; then
    if [[ "${MOCK_HAS_ROUTE:-yes}" == "yes" ]]; then
        printf 'default via 192.168.50.1 proto dhcp src 192.168.50.4 metric 600\n'
    fi
fi
EOF

cat >"${MOCK_BIN}/ping" <<'EOF'
#!/usr/bin/env bash
[[ "${MOCK_BASE_REACHABLE:-yes}" == "yes" ]]
EOF

cat >"${MOCK_BIN}/vcgencmd" <<'EOF'
#!/usr/bin/env bash
printf 'throttled=%s\n' "${MOCK_THROTTLED_HEX:-0x0}"
EOF

chmod +x "${MOCK_BIN}/"*

fail() {
    printf 'FAIL: %s\n' "$1" >&2
    exit 1
}

assert_contains() {
    local output="$1"
    local expected="$2"
    grep -Fqx "$expected" <<<"$output" ||
        fail "expected output line: ${expected}"
}

assert_leds() {
    local red="$1"
    local green="$2"
    local blue="$3"

    [[ "$(<"${LED_ROOT}/wanderer-red/brightness")" == "$red" ]] ||
        fail "unexpected red LED"
    [[ "$(<"${LED_ROOT}/wanderer-green/brightness")" == "$green" ]] ||
        fail "unexpected green LED"
    [[ "$(<"${LED_ROOT}/wanderer-blue/brightness")" == "$blue" ]] ||
        fail "unexpected blue LED"
}

run_health() {
    PATH="${MOCK_BIN}:/usr/bin:/bin" \
    WANDERER_RUN_DIR="$RUN_DIR" \
    WANDERER_LED_ROOT="$LED_ROOT" \
    WANDERER_WIFI_INTERFACE=wlan0 \
    WANDERER_BASE_HOST=192.168.50.2 \
    MOCK_ASSOCIATED="${MOCK_ASSOCIATED:-yes}" \
    MOCK_SIGNAL_DBM="${MOCK_SIGNAL_DBM:--60}" \
    MOCK_HAS_IPV4="${MOCK_HAS_IPV4:-yes}" \
    MOCK_HAS_ROUTE="${MOCK_HAS_ROUTE:-yes}" \
    MOCK_BASE_REACHABLE="${MOCK_BASE_REACHABLE:-yes}" \
    MOCK_THROTTLED_HEX="${MOCK_THROTTLED_HEX:-0x0}" \
        "$HEALTH_SCRIPT" --once
}

run_case() {
    local expected_status="$1"
    local expected_overall="$2"
    local output
    local actual_status

    set +e
    output="$(run_health)"
    actual_status=$?
    set -e

    [[ "$actual_status" == "$expected_status" ]] ||
        fail "expected status ${expected_status}, got ${actual_status}"
    assert_contains "$output" "overall=${expected_overall}"
    printf '%s' "$output"
}

set -e

output="$(run_case 0 OK)"
assert_contains "$output" "wifi_quality=good"
assert_leds 0 1 0

MOCK_SIGNAL_DBM=-70
output="$(run_case 1 WARN)"
assert_contains "$output" "wifi_quality=usable"
assert_leds 1 1 0

MOCK_SIGNAL_DBM=-80
output="$(run_case 2 FAULT)"
assert_contains "$output" "wifi_quality=weak"
assert_leds 1 0 0

MOCK_SIGNAL_DBM=-60
MOCK_THROTTLED_HEX=0x1
output="$(run_case 2 FAULT)"
assert_contains "$output" "undervoltage_now=yes"
assert_leds 1 0 1

MOCK_THROTTLED_HEX=0x0
MOCK_ASSOCIATED=no
MOCK_HAS_IPV4=no
output="$(run_case 2 FAULT)"
assert_contains "$output" "wifi_associated=no"
assert_contains "$output" "ipv4=none"
assert_leds 1 0 0

[[ -s "${RUN_DIR}/health.env" ]] || fail "state file was not written"

printf 'All Wanderer health tests passed.\n'
