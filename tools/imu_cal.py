#!/usr/bin/env python3
"""Guided MinIMU-9 v6 calibration over the Pico USB backdoor.

The tool never edits config.h. It measures one physical quantity and prints
the exact replacement define, so the operator can review, commit, rebuild,
and flash the resulting calibration deliberately.

    python tools/imu_cal.py --check-sign
    python tools/imu_cal.py --scale
    python tools/imu_cal.py --track

`--track` moves the rover in bounded in-place pulses. Keep clear and use a
clear patch of floor; all other modes are hand-motion only.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

from backdoor import Backdoor, find_port, reply_fields


DEG_PER_TURN = 360.0


@dataclass
class ImuReading:
    raw: int
    rate_deg_s: float
    bias_deg_s: float
    psi_deg: float
    healthy: bool


class YawAccumulator:
    """Unwrap the backdoor's (-180, 180] heading field into degrees."""

    def __init__(self, initial_deg: float):
        self._last = initial_deg
        self.degrees = 0.0

    def add(self, psi_deg: float) -> None:
        delta = psi_deg - self._last
        while delta <= -180.0:
            delta += DEG_PER_TURN
        while delta > 180.0:
            delta -= DEG_PER_TURN
        self.degrees += delta
        self._last = psi_deg


def revised_imu_scale(current_scale: float, expected_deg: float,
                      measured_deg: float) -> float:
    """Return the replacement scale while retaining any prior calibration."""
    if expected_deg <= 0.0 or measured_deg <= 0.0:
        raise ValueError("expected and measured angles must be positive")
    return current_scale * expected_deg / measured_deg


def track_width_m(left_ticks: int, right_ticks: int, ticks_per_m: float,
                  yaw_deg: float) -> float:
    """Compute track width from signed encoder travel and integrated yaw."""
    if ticks_per_m <= 0.0:
        raise ValueError("ticks_per_m must be positive")
    yaw_rad = math.radians(yaw_deg)
    if yaw_rad == 0.0:
        raise ValueError("integrated yaw must not be zero")
    return ((left_ticks - right_ticks) / ticks_per_m) / yaw_rad


def fitted_track_width(samples: list[tuple[float, float]]) -> float:
    """Fit travel_m = track_width_m * yaw_rad through cumulative readings."""
    denominator = sum(yaw_rad * yaw_rad for yaw_rad, _ in samples)
    if denominator == 0.0:
        raise ValueError("at least one non-zero yaw sample is required")
    return sum(yaw_rad * travel_m for yaw_rad, travel_m in samples) / denominator


def read_imu(backdoor: Backdoor) -> ImuReading:
    reply = backdoor.request("imu")
    if not reply.startswith("=ok imu"):
        raise RuntimeError(f"imu refused: {reply}")
    fields = reply_fields(reply)
    try:
        return ImuReading(raw=int(fields["raw"]), rate_deg_s=float(fields["rate"]),
                          bias_deg_s=float(fields["bias"]), psi_deg=float(fields["psi"]),
                          healthy=fields["ok"] == "1")
    except (KeyError, ValueError) as exc:
        raise RuntimeError(f"firmware lacks the M2 imu diagnostic: {reply}") from exc


def read_cfg(backdoor: Backdoor) -> dict[str, str]:
    reply = backdoor.request("cfg")
    if not reply.startswith("=ok cfg"):
        raise RuntimeError(f"cfg refused: {reply}")
    fields = reply_fields(reply)
    for name in ("imu_sign", "imu_scale", "ticks_per_m"):
        if name not in fields:
            raise RuntimeError(f"firmware lacks M3 cfg field {name!r}: {reply}")
    return fields


def require_stationary_calibration(backdoor: Backdoor) -> None:
    input("Keep the rover completely still, then press ENTER to calibrate bias. ")
    print("Collecting 512 IMU samples (about five seconds)...")
    reply = backdoor.request("imu cal")
    if not reply.startswith("=ok imu"):
        raise RuntimeError(f"imu cal failed: {reply}")
    fields = reply_fields(reply)
    if fields.get("cal") != "1":
        raise RuntimeError(f"unexpected imu cal reply: {reply}")
    print(f"Bias: {fields['mean']} deg/s, sigma: {fields['sigma']} deg/s")


def sample_for(backdoor: Backdoor, duration_s: float, interval_s: float,
               accumulator: YawAccumulator) -> None:
    deadline = time.monotonic() + duration_s
    while True:
        reading = read_imu(backdoor)
        if not reading.healthy:
            raise RuntimeError("IMU became stale during calibration")
        accumulator.add(reading.psi_deg)
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        time.sleep(min(interval_s, remaining))


def check_sign(backdoor: Backdoor, cfg: dict[str, str], args: argparse.Namespace) -> int:
    require_stationary_calibration(backdoor)
    initial = read_imu(backdoor)
    accumulator = YawAccumulator(initial.psi_deg)
    print(f"\nTurn the rover CLOCKWISE (right) by hand for {args.sign_duration:g} seconds,"
          " starting now.")
    sample_for(backdoor, args.sign_duration, args.interval, accumulator)
    measured = accumulator.degrees
    print(f"Measured right-turn heading: {measured:+.1f} deg")
    if measured <= args.min_motion_deg:
        print("FAIL: right turn was not positive or was too small to judge.")
        print("Set IMU_YAW_SIGN to the opposite sign, rebuild, flash, and retry.")
        return 1
    print(f"PASS: IMU_YAW_SIGN={cfg['imu_sign']} makes a right turn positive.")
    return 0


def scale(backdoor: Backdoor, cfg: dict[str, str], args: argparse.Namespace) -> int:
    require_stationary_calibration(backdoor)
    initial = read_imu(backdoor)
    accumulator = YawAccumulator(initial.psi_deg)
    expected_deg = args.turns * DEG_PER_TURN
    input("\nMark the floor. Press ENTER, then turn the rover CLOCKWISE by hand "
          f"through exactly {args.turns:g} full turns over {args.duration:g} seconds. ")
    sample_for(backdoor, args.duration, args.interval, accumulator)
    measured_deg = accumulator.degrees
    if measured_deg <= 0.0:
        print(f"FAIL: measured {measured_deg:+.1f} deg; use a clockwise/right turn.")
        return 1
    if measured_deg < expected_deg * 0.5:
        print(f"FAIL: only {measured_deg:.1f} deg observed; calibration run was incomplete.")
        return 1

    current_scale = float(cfg["imu_scale"])
    new_scale = revised_imu_scale(current_scale, expected_deg, measured_deg)
    error_pct = (measured_deg / expected_deg - 1.0) * 100.0
    print(f"\nExpected: {expected_deg:.1f} deg; measured: {measured_deg:.1f} deg"
          f" ({error_pct:+.2f}%).")
    print("\nReview, then rebuild and flash:")
    print(f"#define IMU_SCALE  {new_scale:.7f}f")
    return 0


def acquire_dev(backdoor: Backdoor) -> bool:
    reply = backdoor.dev(True)
    if reply.startswith("=ok dev"):
        return True
    print(f"Cannot acquire the backdoor motion lease: {reply}")
    return False


def track(backdoor: Backdoor, cfg: dict[str, str], args: argparse.Namespace) -> int:
    if args.pulse_ms < 1 or args.pulse_ms > 3000:
        print("--pulse-ms must be between 1 and 3000.")
        return 2
    if args.pulses < 1:
        print("--pulses must be positive.")
        return 2
    if abs(args.duty) > backdoor.max_duty:
        print(f"--duty must be within the firmware's +/-{backdoor.max_duty} limit.")
        return 2

    require_stationary_calibration(backdoor)
    print("\nTRACK CALIBRATION MOVES THE ROVER. Put it on clear, level floor, "
          "keep people and cables clear, and mark its initial bearing.")
    if input("Type ROTATE to run bounded in-place pulses: ").strip() != "ROTATE":
        print("Aborted.")
        return 1
    if not acquire_dev(backdoor):
        return 1

    try:
        backdoor.request("enc reset")
        initial = read_imu(backdoor)
        accumulator = YawAccumulator(initial.psi_deg)
        ticks_per_m = float(cfg["ticks_per_m"])
        fit_samples: list[tuple[float, float]] = []
        for pulse in range(args.pulses):
            reply = backdoor.wiggle(args.duty, -args.duty, args.pulse_ms)
            if not reply.startswith("=ok wiggle"):
                raise RuntimeError(f"wiggle refused: {reply}")
            sample_for(backdoor, args.pulse_ms / 1000.0 + 0.1,
                       args.interval, accumulator)
            try:
                backdoor.await_event("!wiggle_done", timeout=2.0)
            except TimeoutError as exc:
                raise RuntimeError("wiggle completion was not reported") from exc
            left_ticks, right_ticks = backdoor.enc()
            yaw_rad = math.radians(accumulator.degrees)
            fit_samples.append((yaw_rad, (left_ticks - right_ticks) / ticks_per_m))
            print(f"  pulse {pulse + 1}/{args.pulses}: "
                  f"heading {accumulator.degrees / DEG_PER_TURN:+.2f} turns")

        left_ticks, right_ticks = backdoor.enc()
    finally:
        backdoor.dev(False)

    yaw_rad = math.radians(accumulator.degrees)
    min_yaw_rad = args.min_turns * 2.0 * math.pi
    if abs(yaw_rad) < min_yaw_rad:
        print(f"FAIL: only {abs(accumulator.degrees) / DEG_PER_TURN:.2f} turns; "
              f"need at least {args.min_turns:g}. Increase --pulses or --duty.")
        return 1

    track_m = fitted_track_width(fit_samples)
    if track_m <= 0.0:
        print("FAIL: encoder and gyro turn signs disagree; do not apply this value.")
        return 1
    print(f"\nEncoder delta: left={left_ticks} right={right_ticks}")
    print(f"Integrated yaw: {math.degrees(yaw_rad):.1f} deg")
    print("\nReview, then rebuild and flash:")
    print(f"#define TRACK_WIDTH_M  {track_m:.7f}f")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Pico USB CDC port (default: autodetect)")
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument("--check-sign", action="store_true",
                         help="verify clockwise/right hand rotation is positive")
    actions.add_argument("--scale", action="store_true",
                         help="measure IMU_SCALE over hand-turned full rotations")
    actions.add_argument("--track", action="store_true",
                         help="measure TRACK_WIDTH_M with bounded in-place pulses")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="seconds allowed for the --scale hand rotation (default: 30)")
    parser.add_argument("--sign-duration", type=float, default=3.0,
                        help="seconds allowed for the --check-sign turn (default: 3)")
    parser.add_argument("--interval", type=float, default=0.05,
                        help="IMU sampling interval in seconds (default: 0.05)")
    parser.add_argument("--turns", type=float, default=5.0,
                        help="known hand turns for --scale (default: 5)")
    parser.add_argument("--min-motion-deg", type=float, default=20.0,
                        help="minimum heading change for --check-sign (default: 20)")
    parser.add_argument("--duty", type=int, default=250,
                        help="per-mille duty for --track right rotation (default: 250)")
    parser.add_argument("--pulse-ms", type=int, default=500,
                        help="bounded pulse length for --track (default: 500)")
    parser.add_argument("--pulses", type=int, default=4,
                        help="number of in-place pulses for --track (default: 4)")
    parser.add_argument("--min-turns", type=float, default=0.25,
                        help="minimum observed turns for --track (default: 0.25)")
    args = parser.parse_args()
    if (args.duration <= 0.0 or args.sign_duration <= 0.0
            or args.interval <= 0.0 or args.turns <= 0.0):
        parser.error("--duration, --sign-duration, --interval, and --turns must be positive")

    port = find_port(args.port)
    print(f"Opening {port}")
    backdoor = Backdoor(port)
    try:
        print(f"Airframe: {backdoor.ver()}")
        cfg = read_cfg(backdoor)
        print(f"Configured IMU: sign={cfg['imu_sign']} scale={cfg['imu_scale']}")
        if args.check_sign:
            return check_sign(backdoor, cfg, args)
        if args.scale:
            return scale(backdoor, cfg, args)
        return track(backdoor, cfg, args)
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        backdoor.close()


if __name__ == "__main__":
    sys.exit(main())
