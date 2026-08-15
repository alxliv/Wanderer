#!/usr/bin/env python3
"""Read-only cockpit IMU and encoder sign monitor.

Run on the Pi, from ``pilot/``::

    python3 -m helm.imu_monitor

No motor command is sent. With the rover unarmed, turn it by hand and roll it
by hand. The expected conventions are:

* clockwise/right viewed from above: positive ``rate`` and positive ``dpsi``;
* counter-clockwise/left: negative ``rate`` and negative ``dpsi``;
* pushing the rover forward: positive left and right encoder deltas;
* pulling the rover backward: negative left and right encoder deltas.

For an unambiguous encoder test, raise the driven wheels and rotate each wheel
forward by hand: the left wheel must change only ``dL`` and the right only
``dR``. A ground push can skid tyres or fail to back-drive a gearbox.

Use ``--zero`` only when the rover is still, to make the displayed heading
relative to that physical orientation. Ctrl-C exits without changing state.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

# Runnable from pilot/ as either `python3 -m helm.imu_monitor` or
# `python3 helm/imu_monitor.py`, matching helm.py.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cockpit import Cockpit, CockpitTimeout
from cockpit.uart_link import UartCockpitLink

if __package__:
    from . import presets
else:
    import presets


def wrap_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def motion_label(rate_rad_s: float, left_delta: int, right_delta: int) -> str:
    """Human-readable direction verdict for one sampled interval."""
    if rate_rad_s > 0.05:
        turn = "RIGHT/CW"
    elif rate_rad_s < -0.05:
        turn = "LEFT/CCW"
    else:
        turn = "still"

    total = left_delta + right_delta
    if total > 0:
        roll = "FORWARD"
    elif total < 0:
        roll = "REVERSE"
    else:
        roll = "still"
    return f"turn={turn:<9} roll={roll}"


def wait_for_airframe_startup(cockpit: Cockpit) -> None:
    deadline = time.monotonic() + presets.AIRFRAME_STARTUP_TIMEOUT_S
    while True:
        try:
            cockpit.ping()
            return
        except CockpitTimeout:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise
            time.sleep(min(presets.AIRFRAME_STARTUP_RETRY_PERIOD_S, remaining))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/serial0",
                        help="cockpit UART device (default: /dev/serial0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--period", type=float, default=0.20,
                        help="seconds between readings (default: 0.20; 5 Hz)")
    parser.add_argument("--zero", action="store_true",
                        help="zero heading once at startup; rover must be still")
    args = parser.parse_args(argv)
    if not 0.02 <= args.period <= 2.0:
        parser.error("--period must be between 0.02 and 2 seconds")

    with Cockpit(UartCockpitLink(args.port, args.baud), command_timeout=0.30) as cockpit:
        wait_for_airframe_startup(cockpit)
        if args.zero:
            cockpit.zero_heading()

        heading = cockpit.heading()
        odometry = cockpit.odometry()
        reference_heading = heading.psi_rad
        previous_heading = heading.psi_rad
        previous_left = odometry.left_ticks
        previous_right = odometry.right_ticks
        print("Monitoring only: no motor command has been sent.")
        print("Clockwise/right by hand must show +rate/+dpsi; pushing forward must show +dL/+dR.")
        print("  elapsed    psi   dpsi    rate    bias valid     dL     dR       left      right  verdict")
        started = time.monotonic()
        try:
            while True:
                heading = cockpit.heading()
                odometry = cockpit.odometry()
                dpsi = wrap_pi(heading.psi_rad - previous_heading)
                relative_psi = wrap_pi(heading.psi_rad - reference_heading)
                dl = odometry.left_ticks - previous_left
                dr = odometry.right_ticks - previous_right
                print(f"{time.monotonic() - started:9.2f} "
                      f"{math.degrees(relative_psi):+6.1f} "
                      f"{math.degrees(dpsi):+6.2f} "
                      f"{math.degrees(heading.rate_rad_s):+7.2f} "
                      f"{math.degrees(heading.bias_rad_s):+7.2f} "
                      f"  {int(heading.valid)}  {dl:+6d} {dr:+6d} "
                      f"{odometry.left_ticks:+10d} {odometry.right_ticks:+10d}  "
                      f"{motion_label(heading.rate_rad_s, dl, dr)}",
                      flush=True)
                previous_heading = heading.psi_rad
                previous_left = odometry.left_ticks
                previous_right = odometry.right_ticks
                time.sleep(args.period)
        except KeyboardInterrupt:
            print("\nMonitor stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
