import unittest

from cockpit import MoveStatus
from helm.move_trace import TraceSample, console_row, csv_row, saturation_label


class MoveTraceTest(unittest.TestCase):
    def sample(self, saturation=0):
        return TraceSample(
            status=MoveStatus(
                active=True, elapsed_s=0.2, heading_ref_rad=0.0,
                heading_rad=0.05, error_rad=-0.05, rate_rad_s=0.1,
                p_rad_s=-0.1, i_rad_s=-0.005, d_rad_s=-0.01,
                omega_rad_s=-0.115, left_m_s=0.138, right_m_s=0.162,
                saturation=saturation),
            left_ticks=120, right_ticks=140,
            left_travel_m=0.012, right_travel_m=0.014)

    def test_saturation_labels_decode_both_bits(self):
        self.assertEqual(saturation_label(0), "-")
        self.assertEqual(saturation_label(1), "omega")
        self.assertEqual(saturation_label(2), "wheel")
        self.assertEqual(saturation_label(3), "omega+wheel")

    def test_console_row_uses_human_units(self):
        row = console_row(self.sample(saturation=1))
        self.assertIn("-2.86", row)  # -0.05 rad in degrees
        self.assertIn("omega", row)
        self.assertIn("+0.012", row)

    def test_csv_row_keeps_si_units_and_raw_ticks(self):
        row = csv_row(self.sample())
        self.assertEqual(row["error_rad"], "-0.050000")
        self.assertEqual(row["left_target_m_s"], "0.138000")
        self.assertEqual(row["left_ticks"], 120)


if __name__ == "__main__":
    unittest.main()
