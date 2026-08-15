import math
import unittest

from helm.motor_gain_cal import estimate_right_gain, wrap_pi


class MotorGainMathTest(unittest.TestCase):
    def test_right_turn_increases_right_gain(self):
        result = estimate_right_gain(current_right_gain=841,
                                     left_tick_delta=10000,
                                     right_tick_delta=10000,
                                     ticks_per_meter=10000.0,
                                     track_m=0.2,
                                     heading_change_rad=0.1)
        self.assertAlmostEqual(result.mean_travel_m, 1.0)
        self.assertAlmostEqual(result.left_travel_m, 1.01)
        self.assertAlmostEqual(result.right_travel_m, 0.99)
        self.assertEqual(result.proposed_right_gain, 858)

    def test_left_turn_decreases_right_gain(self):
        result = estimate_right_gain(current_right_gain=841,
                                     left_tick_delta=10000,
                                     right_tick_delta=10000,
                                     ticks_per_meter=10000.0,
                                     track_m=0.2,
                                     heading_change_rad=-0.1)
        self.assertEqual(result.proposed_right_gain, 824)

    def test_invalid_forward_roll_is_rejected(self):
        with self.assertRaises(ValueError):
            estimate_right_gain(current_right_gain=841,
                                left_tick_delta=-1, right_tick_delta=-1,
                                ticks_per_meter=10000.0, track_m=0.2,
                                heading_change_rad=0.0)

    def test_heading_wrap(self):
        self.assertAlmostEqual(wrap_pi(-math.pi + 0.1 - (math.pi - 0.1)), 0.2)



if __name__ == "__main__":
    unittest.main()
