import math
import unittest

from cockpit.errors import CockpitTimeout
from helm.motor_gain_cal import (estimate_gain_pair, normalized_gain_pair,
                                 wait_for_airframe_startup, wrap_pi)


class MotorGainMathTest(unittest.TestCase):
    def test_right_turn_increases_right_gain(self):
        result = estimate_gain_pair(current_left_gain=1000,
                                    current_right_gain=841,
                                    left_tick_delta=10000,
                                    right_tick_delta=10000,
                                    ticks_per_meter=10000.0,
                                    track_m=0.2,
                                    heading_change_rad=0.1)
        self.assertAlmostEqual(result.mean_travel_m, 1.0)
        self.assertAlmostEqual(result.left_travel_m, 1.01)
        self.assertAlmostEqual(result.right_travel_m, 0.99)
        self.assertEqual((result.proposed_left_gain,
                          result.proposed_right_gain), (1000, 858))

    def test_left_turn_decreases_right_gain(self):
        result = estimate_gain_pair(current_left_gain=1000,
                                    current_right_gain=841,
                                    left_tick_delta=10000,
                                    right_tick_delta=10000,
                                    ticks_per_meter=10000.0,
                                    track_m=0.2,
                                    heading_change_rad=-0.1)
        self.assertEqual((result.proposed_left_gain,
                          result.proposed_right_gain), (1000, 824))

    def test_required_right_boost_derates_left_instead(self):
        result = estimate_gain_pair(current_left_gain=1000,
                                    current_right_gain=841,
                                    left_tick_delta=10000,
                                    right_tick_delta=10000,
                                    ticks_per_meter=10000.0,
                                    track_m=0.2,
                                    heading_change_rad=1.0)
        self.assertEqual(result.proposed_right_gain, 1000)
        self.assertLess(result.proposed_left_gain, 1000)
        self.assertAlmostEqual(result.right_to_left_gain_ratio,
                               result.proposed_right_gain / result.proposed_left_gain,
                               places=3)

    def test_normalization_keeps_weaker_side_at_full_gain(self):
        self.assertEqual(normalized_gain_pair(0.8), (1000, 800))
        self.assertEqual(normalized_gain_pair(1.25), (800, 1000))

    def test_invalid_forward_roll_is_rejected(self):
        with self.assertRaises(ValueError):
            estimate_gain_pair(current_left_gain=1000,
                               current_right_gain=841,
                               left_tick_delta=-1, right_tick_delta=-1,
                               ticks_per_meter=10000.0, track_m=0.2,
                               heading_change_rad=0.0)

    def test_heading_wrap(self):
        self.assertAlmostEqual(wrap_pi(-math.pi + 0.1 - (math.pi - 0.1)), 0.2)

    def test_startup_retries_ping(self):
        class StartingCockpit:
            def __init__(self):
                self.count = 0

            def ping(self):
                self.count += 1
                if self.count < 3:
                    raise CockpitTimeout("still starting")

        cockpit = StartingCockpit()
        wait_for_airframe_startup(cockpit)
        self.assertEqual(cockpit.count, 3)



if __name__ == "__main__":
    unittest.main()
