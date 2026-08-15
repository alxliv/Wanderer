#!/usr/bin/env python3
"""Focused offline checks for the IMU calibration calculations."""

import math
import unittest

from imu_cal import (YawAccumulator, fitted_track_width, revised_imu_scale,
                     track_width_m)


class ImuCalibrationTests(unittest.TestCase):
    def test_yaw_accumulator_unwraps_forward_crossing(self) -> None:
        yaw = YawAccumulator(170.0)
        yaw.add(-170.0)
        yaw.add(-160.0)
        self.assertAlmostEqual(yaw.degrees, 30.0)

    def test_scale_retains_prior_calibration(self) -> None:
        self.assertAlmostEqual(revised_imu_scale(1.02, 3600.0, 3500.0),
                               1.02 * 3600.0 / 3500.0)

    def test_track_width_uses_signed_encoder_difference(self) -> None:
        width = track_width_m(3831, -3831, 3831.0, 360.0)
        self.assertAlmostEqual(width, 2.0 / (2.0 * math.pi))

    def test_track_width_rejects_zero_yaw(self) -> None:
        with self.assertRaises(ValueError):
            track_width_m(10, -10, 100.0, 0.0)

    def test_track_fit_uses_multiple_cumulative_readings(self) -> None:
        self.assertAlmostEqual(fitted_track_width([
            (2.0 * math.pi, 0.20 * 2.0 * math.pi),
            (4.0 * math.pi, 0.20 * 4.0 * math.pi),
            (6.0 * math.pi, 0.20 * 6.0 * math.pi),
        ]), 0.20)


if __name__ == "__main__":
    unittest.main()
