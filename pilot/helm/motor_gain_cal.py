#!/usr/bin/env python3
"""Ground-load calibration for ``MOTOR_RIGHT_GAIN_PERMILLE``.

Run on the Pi, from ``pilot/``::

    python3 -m helm.motor_gain_cal --speed 0.25

The tool performs short *open-loop* straight-drive runs over the cockpit UART.
It must not use ``proc move``: that procedure holds heading and would conceal
the exact wheel mismatch this tool is measuring.  For each run it measures the
gyro heading change and mean encoder travel, derives the left/right travel
ratio, and prints a median replacement gain.  It never edits or flashes
``config.h`` itself.
"""

from __future__ import annotations

import argparse
import math
import statistics
import time
from dataclasses import dataclass

from cockpit import Cockpit, CockpitError, TacticalState
from cockpit.uart_link import UartCockpitLink


@dataclass(frozen=True)
class RunMeasurement:
    """One open-loop forward run, expressed in airframe-owned SI units."""

    mean_travel_m: float
    heading_change_rad: float
    left_travel_m: float
    right_travel_m: float
    proposed_right_gain: int


def wrap_pi(angle_rad: float) -> float:
    """Shortest signed angular difference in ``[-pi, pi)``."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def estimate_right_gain(*, current_right_gain: int, left_tick_delta: int,
                         right_tick_delta: int, ticks_per_meter: float,
                         track_m: float, heading_change_rad: float) -> RunMeasurement:
    """Estimate the right gain which equalizes a forward ground run.

    The gyro supplies the physical differential travel:
    ``left - right = track * heading_change``.  The encoders supply the mean
    signed travel.  Combining those two measurements avoids treating a small
    left/right encoder scale difference as motor mismatch.
    """
    if current_right_gain <= 0:
        raise ValueError("current right gain must be positive")
    if ticks_per_meter <= 0.0 or track_m <= 0.0:
        raise ValueError("airframe geometry is invalid")

    mean_m = (left_tick_delta + right_tick_delta) / (2.0 * ticks_per_meter)
    differential_m = track_m * heading_change_rad
    left_m = mean_m + differential_m / 2.0
    right_m = mean_m - differential_m / 2.0
    if mean_m <= 0.0 or left_m <= 0.0 or right_m <= 0.0:
        raise ValueError("run was not a usable forward roll")

    gain = round(current_right_gain * left_m / right_m)
    if not 1 <= gain <= 1000:
        raise ValueError("estimated gain is outside the valid 1..1000 range")
    return RunMeasurement(mean_travel_m=mean_m,
                          heading_change_rad=heading_change_rad,
                          left_travel_m=left_m,
                          right_travel_m=right_m,
                          proposed_right_gain=gain)


def require_drive_confirmation(run_number: int, runs: int) -> None:
    answer = input(
        f"\nRun {run_number}/{runs}: rover on clear level floor, pointed safely "
        "forward, with people and cables clear. Type DRIVE to begin: ")
    if answer.strip() != "DRIVE":
        raise KeyboardInterrupt


def run_once(cockpit: Cockpit, *, speed_m_s: float, duration_s: float,
             settle_s: float, current_right_gain: int) -> RunMeasurement:
    geometry = cockpit.geometry()
    before_heading = cockpit.heading()
    before_odometry = cockpit.odometry()
    if not before_heading.valid:
        raise RuntimeError("IMU heading is not valid; do not calibrate")

    deadline = time.monotonic() + duration_s
    while True:
        cockpit.drive(speed_m_s, 0.0)
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        time.sleep(min(0.10, remaining))
    cockpit.stop()
    time.sleep(settle_s)

    after_heading = cockpit.heading()
    after_odometry = cockpit.odometry()
    if not after_heading.valid:
        raise RuntimeError("IMU heading became invalid during the run")

    return estimate_right_gain(
        current_right_gain=current_right_gain,
        left_tick_delta=after_odometry.left_ticks - before_odometry.left_ticks,
        right_tick_delta=after_odometry.right_ticks - before_odometry.right_ticks,
        ticks_per_meter=geometry.ticks_per_meter,
        track_m=geometry.track_m,
        heading_change_rad=wrap_pi(after_heading.psi_rad - before_heading.psi_rad))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/serial0",
                        help="cockpit UART device (default: /dev/serial0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--speed", type=float, default=0.25,
                        help="forward calibration speed in m/s (default: 0.25)")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="seconds per run (default: 2.0)")
    parser.add_argument("--runs", type=int, default=3,
                        help="number of forward runs; median is used (default: 3)")
    parser.add_argument("--settle", type=float, default=0.30,
                        help="seconds stationary before the final reading (default: 0.30)")
    args = parser.parse_args(argv)

    if not 0.05 <= args.speed <= 0.30:
        parser.error("--speed must be between 0.05 and 0.30 m/s")
    if not 0.25 <= args.duration <= 5.0:
        parser.error("--duration must be between 0.25 and 5 seconds")
    if not 1 <= args.runs <= 9:
        parser.error("--runs must be between 1 and 9")
    if args.settle < 0.0:
        parser.error("--settle must not be negative")

    expected_m = args.speed * args.duration * args.runs
    print("GROUND-LOAD MOTOR GAIN CALIBRATION")
    print(f"This makes {args.runs} open-loop forward runs of about "
          f"{args.speed * args.duration:.2f} m each ({expected_m:.2f} m total).")
    print("Do not run Helm or any other cockpit client at the same time.")
    print("The rover will stop after every run; Ctrl-C sends stop then disarm.")

    measurements: list[RunMeasurement] = []
    link = UartCockpitLink(args.port, args.baud)
    with Cockpit(link, command_timeout=0.30) as cockpit:
        try:
            cockpit.arm()
            if cockpit.state() != TacticalState.ACTIVE:
                raise RuntimeError("airframe is not ACTIVE; disarm then arm it again")
            motor_config = cockpit.motor_config()
            left_gain = motor_config.left_gain_permille
            right_gain = motor_config.right_gain_permille
            print(f"Pico reports: left gain={left_gain}, right gain={right_gain}; "
                  f"deadbands={motor_config.left_deadband_permille}/"
                  f"{motor_config.right_deadband_permille}.")
            for index in range(1, args.runs + 1):
                require_drive_confirmation(index, args.runs)
                measurement = run_once(cockpit, speed_m_s=args.speed,
                                       duration_s=args.duration,
                                       settle_s=args.settle,
                                       current_right_gain=right_gain)
                measurements.append(measurement)
                print(f"  heading={math.degrees(measurement.heading_change_rad):+.2f} deg"
                      f"  travel L/R={measurement.left_travel_m:.3f}/"
                      f"{measurement.right_travel_m:.3f} m"
                      f"  proposed right gain={measurement.proposed_right_gain}")
        except KeyboardInterrupt:
            print("\nAborted.")
            return 130
        except (CockpitError, RuntimeError, ValueError) as exc:
            print(f"\nCalibration failed: {exc}")
            return 1
        finally:
            try:
                cockpit.stop()
            except CockpitError:
                pass
            try:
                cockpit.disarm()
            except CockpitError:
                pass

    proposal = round(statistics.median(m.proposed_right_gain for m in measurements))
    headings = [math.degrees(m.heading_change_rad) for m in measurements]
    print("\nRESULT")
    print(f"  current gains:      left={left_gain}, right={right_gain}")
    print(f"  heading changes:    {', '.join(f'{value:+.2f}' for value in headings)} deg")
    print(f"  median proposal:    {proposal}")
    print("\nReview, then rebuild and flash:")
    print(f"#define MOTOR_LEFT_GAIN_PERMILLE    {left_gain}  /* reference */")
    print(f"#define MOTOR_RIGHT_GAIN_PERMILLE   {proposal}")
    print("\nRepeat this calibration once after flashing. Then run `helm` ->"
          " `speed 0.25`, `move 3` to validate M6 heading hold.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
