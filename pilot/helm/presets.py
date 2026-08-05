"""Helm presets — bench-calibration constants, all subject to tuning.

These are *console* settings (how the helm drives the cockpit API), not
vehicle properties: drivetrain geometry is queried from the airframe at
startup (get_geometry) and performance limits are the airframe's own
business (it scales over-limit drive requests itself).
"""

# Telegraph speeds, m/s. 'full' is the anchor; the rest derive from it.
FULL_SPEED = 1.0
HALF_SPEED = FULL_SPEED / 2
SLOW_SPEED = FULL_SPEED / 5

# 'turn <deg>' maneuver: rotation rate used, and the speed cap applied
# while turning (current speed above the cap is reduced to it and NOT
# restored afterwards — you resume at the slowed speed).
TURN_RATE_DPS = 30.0
TURN_SPEED = 0.2

# The placeholder turn loop commands rotation-stop this many degrees early
# to absorb poll latency and coast. Tune on the bench until 'turn 90'
# actually lands near 90.
OVERSHOOT_COMP_DEG = 2.0

# Streaming: the helm holds the latched {speed, bank} setpoint true by
# sending 'drive' at this period while engaged (well inside the airframe's
# 750 ms deadman), and 'ping' at the idle period otherwise.
STREAM_PERIOD_S = 0.1
IDLE_PING_PERIOD_S = 0.25

# Where airframe events and helm notices are appended (second window:
#   tail -f ~/wanderer/events.log ).
EVENT_LOG = "~/wanderer/events.log"

# Default cockpit UART device for bench runs on the RPi5.
SERIAL_DEVICE = "/dev/serial0"
