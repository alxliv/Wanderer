import unittest

from helm.imu_monitor import motion_label, wrap_pi


class ImuMonitorTest(unittest.TestCase):
    def test_turn_labels_follow_cockpit_convention(self):
        self.assertIn("RIGHT/CW", motion_label(0.1, 0, 0))
        self.assertIn("LEFT/CCW", motion_label(-0.1, 0, 0))

    def test_roll_labels_follow_encoder_convention(self):
        self.assertIn("FORWARD", motion_label(0.0, 4, 3))
        self.assertIn("REVERSE", motion_label(0.0, -4, -3))

    def test_heading_delta_wraps(self):
        self.assertAlmostEqual(wrap_pi(-3.0 - 3.0), 0.283185307, places=6)


if __name__ == "__main__":
    unittest.main()
