#!/usr/bin/env python3
"""Trace one firmware-owned M6 move on the cockpit UART.

Run on the Pi, from ``pilot/``::

    python3 -m helm.move_trace --distance 0.5 --speed 0.15

The tool is the sole cockpit client. It starts ``proc move``, pulls the exact
controller sample and odometry at 5 Hz, prints a readable table, and saves the
same samples as SI-unit CSV. Positive heading/rate/omega is clockwise/right.
Ctrl-C aborts the procedure, stops, and disarms the rover.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import queue
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# Runnable from pilot/ as either `python3 -m helm.move_trace` or
# `python3 helm/move_trace.py`, matching helm.py.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cockpit import (Cockpit, CockpitError, CockpitTimeout, MoveStatus,
                     ProcedureFinished, TacticalState)
from cockpit.uart_link import UartCockpitLink

if __package__:
    from . import presets
else:
    import presets


MIN_TELEMETRY_VERSION = (0, 16)


@dataclass(frozen=True)
class TraceSample:
    status: MoveStatus
    left_ticks: int
    right_ticks: int
    left_travel_m: float
    right_travel_m: float


def saturation_label(mask: int) -> str:
    """Short display name for the firmware saturation bit mask."""
    labels = []
    if mask & 1:
        labels.append("omega")
    if mask & 2:
        labels.append("wheel")
    return "+".join(labels) if labels else "-"


def console_row(sample: TraceSample) -> str:
    """Format one trace row; angles are human-readable degrees."""
    s = sample.status
    degrees = math.degrees
    return (f"{s.elapsed_s:7.2f} {int(s.active):1d} "
            f"{degrees(s.heading_rad):+7.2f} {degrees(s.error_rad):+7.2f} "
            f"{degrees(s.rate_rad_s):+7.2f} "
            f"{degrees(s.p_rad_s):+7.2f} {degrees(s.i_rad_s):+7.2f} "
            f"{degrees(s.d_rad_s):+7.2f} {degrees(s.omega_rad_s):+7.2f} "
            f"{s.left_m_s:+6.3f} {s.right_m_s:+6.3f} "
            f"{saturation_label(s.saturation):>11} "
            f"{sample.left_travel_m:+7.3f} {sample.right_travel_m:+7.3f}")


def wait_for_airframe_startup(cockpit: Cockpit) -> None:
    """Tolerate the Pico boot window before its UART handler is ready."""
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


def capture_sample(cockpit: Cockpit, *, start_left: int, start_right: int,
                   ticks_per_meter: float) -> TraceSample:
    status = cockpit.move_status()
    odometry = cockpit.odometry()
    return TraceSample(
        status=status,
        left_ticks=odometry.left_ticks,
        right_ticks=odometry.right_ticks,
        left_travel_m=(odometry.left_ticks - start_left) / ticks_per_meter,
        right_travel_m=(odometry.right_ticks - start_right) / ticks_per_meter)


CSV_FIELDS = (
    "elapsed_s", "active", "heading_ref_rad", "heading_rad", "error_rad",
    "rate_rad_s", "p_rad_s", "i_rad_s", "d_rad_s", "omega_rad_s",
    "left_target_m_s", "right_target_m_s", "saturation",
    "left_ticks", "right_ticks", "left_travel_m", "right_travel_m",
)


def csv_row(sample: TraceSample) -> dict[str, object]:
    s = sample.status
    return {
        "elapsed_s": f"{s.elapsed_s:.3f}",
        "active": int(s.active),
        "heading_ref_rad": f"{s.heading_ref_rad:.6f}",
        "heading_rad": f"{s.heading_rad:.6f}",
        "error_rad": f"{s.error_rad:.6f}",
        "rate_rad_s": f"{s.rate_rad_s:.6f}",
        "p_rad_s": f"{s.p_rad_s:.6f}",
        "i_rad_s": f"{s.i_rad_s:.6f}",
        "d_rad_s": f"{s.d_rad_s:.6f}",
        "omega_rad_s": f"{s.omega_rad_s:.6f}",
        "left_target_m_s": f"{s.left_m_s:.6f}",
        "right_target_m_s": f"{s.right_m_s:.6f}",
        "saturation": s.saturation,
        "left_ticks": sample.left_ticks,
        "right_ticks": sample.right_ticks,
        "left_travel_m": f"{sample.left_travel_m:.6f}",
        "right_travel_m": f"{sample.right_travel_m:.6f}",
    }


def safe_stop(cockpit: Cockpit) -> None:
    """Best-effort cleanup for every normal and exceptional exit."""
    for command in (cockpit.abort, cockpit.stop, cockpit.disarm):
        try:
            command()
        except CockpitError:
            pass


def default_output_path() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return Path(f"move_trace_{timestamp}.csv")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/serial0",
                        help="cockpit UART device (default: /dev/serial0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--distance", type=float, default=0.5,
                        help="signed move distance in metres (default: 0.5)")
    parser.add_argument("--speed", type=float, default=0.15,
                        help="positive speed magnitude in m/s (default: 0.15)")
    parser.add_argument("--period", type=float, default=0.20,
                        help="sample period in seconds (default: 0.20; 5 Hz)")
    parser.add_argument("--output", type=Path,
                        help="CSV path (default: timestamped file in current directory)")
    args = parser.parse_args(argv)

    if not math.isfinite(args.distance) or not 0.1 <= abs(args.distance) <= 3.0:
        parser.error("--distance magnitude must be between 0.1 and 3.0 m")
    if not math.isfinite(args.speed) or not 0.05 <= args.speed <= 0.30:
        parser.error("--speed must be between 0.05 and 0.30 m/s")
    if not math.isfinite(args.period) or not 0.10 <= args.period <= 0.25:
        parser.error("--period must be between 0.10 and 0.25 seconds")

    output_path = args.output or default_output_path()
    linear_m_s = math.copysign(args.speed, args.distance)
    print("M6 CLOSED-LOOP MOVE TRACE")
    print("Do not run Helm or another cockpit client at the same time.")
    print("Keep people and the USB cable clear. Ctrl-C stops and disarms.")

    cockpit = Cockpit(UartCockpitLink(args.port, args.baud), command_timeout=0.30)
    samples: list[TraceSample] = []
    result: ProcedureFinished | None = None
    try:
        with cockpit:
            try:
                wait_for_airframe_startup(cockpit)
                version = cockpit.version()
                if (version.major, version.minor) < MIN_TELEMETRY_VERSION:
                    raise RuntimeError(
                        f"airframe fw {version.major}.{version.minor} lacks move telemetry; "
                        "build and flash fw 0.16 or newer")

                state = cockpit.state()
                if state == TacticalState.FAULT:
                    raise RuntimeError("airframe is in FAULT; clear it before tracing")
                if state == TacticalState.ACTIVE:
                    cockpit.stop()
                    cockpit.disarm()
                elif state == TacticalState.FALLBACK:
                    cockpit.disarm()

                geometry = cockpit.geometry()
                motor = cockpit.motor_config()
                heading = cockpit.heading()
                if not heading.valid:
                    raise RuntimeError("IMU heading is not valid")

                print(f"Pico fw {version.major}.{version.minor}; gains "
                      f"L/R={motor.left_gain_permille}/{motor.right_gain_permille}; "
                      f"deadbands={motor.left_deadband_permille}/"
                      f"{motor.right_deadband_permille}.")
                print(f"Will move {args.distance:+.2f} m at {linear_m_s:+.2f} m/s "
                      f"and sample every {args.period:.2f} s.")
                answer = input("Place the rover safely and type MOVE to begin: ")
                if answer.strip() != "MOVE":
                    print("Cancelled.")
                    return 0

                # Refuse to overwrite a prior diagnostic trace.
                with output_path.open("x", newline="", encoding="utf-8") as csv_file:
                    writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
                    writer.writeheader()

                    outcomes: "queue.Queue[ProcedureFinished]" = queue.Queue()

                    def on_procedure(event: ProcedureFinished) -> None:
                        if event.name == "move":
                            outcomes.put(event)

                    unsubscribe = cockpit.on_event(on_procedure, ProcedureFinished)
                    try:
                        cockpit.arm()
                        if cockpit.state() != TacticalState.ACTIVE:
                            raise RuntimeError("airframe did not enter ACTIVE")
                        before = cockpit.odometry()
                        started = cockpit.start_move(args.distance, linear_m_s)
                        deadline = time.monotonic() + started.timeout_s + 1.0
                        next_sample = time.monotonic()

                        print("\nUnits: psi/error in deg; rate/P/I/D/omega in deg/s; "
                              "targets in m/s; travel in m.")
                        print("   time A     psi     err    rate       P       I       D   omega "
                              "  tgtL   tgtR         sat   travelL travelR")

                        while result is None:
                            sample = capture_sample(
                                cockpit, start_left=before.left_ticks,
                                start_right=before.right_ticks,
                                ticks_per_meter=geometry.ticks_per_meter)
                            samples.append(sample)
                            writer.writerow(csv_row(sample))
                            csv_file.flush()
                            print(console_row(sample), flush=True)

                            next_sample += args.period
                            wait_s = min(max(0.0, next_sample - time.monotonic()),
                                         max(0.0, deadline - time.monotonic()))
                            try:
                                result = outcomes.get(timeout=wait_s)
                            except queue.Empty:
                                if time.monotonic() >= deadline:
                                    cockpit.abort()
                                    raise RuntimeError("procedure outcome timed out")

                        # Capture stopped/final encoder state after the outcome.
                        final_sample = capture_sample(
                            cockpit, start_left=before.left_ticks,
                            start_right=before.right_ticks,
                            ticks_per_meter=geometry.ticks_per_meter)
                        samples.append(final_sample)
                        writer.writerow(csv_row(final_sample))
                        csv_file.flush()
                        print(console_row(final_sample), flush=True)
                    finally:
                        unsubscribe()
            except KeyboardInterrupt:
                print("\nTrace aborted by operator.")
                return 130
            finally:
                safe_stop(cockpit)
    except (CockpitError, OSError, RuntimeError) as exc:
        print(f"\nMove trace failed: {exc}")
        return 1

    assert result is not None and samples
    final = samples[-1]
    peak_error = max(abs(math.degrees(s.status.error_rad)) for s in samples)
    peak_omega = max(abs(math.degrees(s.status.omega_rad_s)) for s in samples)
    omega_saturated = sum(bool(s.status.saturation & 1) for s in samples)
    wheel_saturated = sum(bool(s.status.saturation & 2) for s in samples)
    mean_travel = (final.left_travel_m + final.right_travel_m) / 2.0
    detail = f" ({result.reason})" if result.reason else ""
    print("\nRESULT")
    print(f"  outcome:              {result.outcome}{detail}")
    print(f"  mean encoder travel:  {mean_travel:+.3f} m")
    print(f"  final heading error:  {math.degrees(final.status.error_rad):+.2f} deg")
    print(f"  peak heading error:   {peak_error:.2f} deg")
    print(f"  peak correction:      {peak_omega:.2f} deg/s")
    print(f"  saturated samples:    omega={omega_saturated}, wheel={wheel_saturated}")
    print(f"  CSV:                  {output_path}")
    return 0 if result.outcome == "DONE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
