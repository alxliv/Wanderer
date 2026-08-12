import contextlib
import io
import tempfile
import unittest

from cockpit.errors import CockpitTimeout
from cockpit.events import TacticalState
from helm.helm import Helm


class StartupCockpit:
    def __init__(self):
        self.ping_count = 0
        self.ping_count_at_version = None

    def on_event(self, handler):
        pass

    def ping(self):
        self.ping_count += 1
        if self.ping_count < 3:
            raise CockpitTimeout("airframe still starting")

    def version(self):
        self.ping_count_at_version = self.ping_count
        return type("Version", (), {"major": 0, "minor": 3})()

    def geometry(self):
        return type("Geometry", (), {
            "ticks_per_meter": 3831.0,
            "track_m": 0.3,
        })()

    def state(self):
        return TacticalState.SAFE


class HelmStartupTests(unittest.TestCase):
    def test_start_retries_ping_while_airframe_starts(self):
        cockpit = StartupCockpit()
        with tempfile.TemporaryDirectory() as temp_dir:
            helm = Helm(cockpit, temp_dir + "/events.log")
            helm.start()
            helm.shutdown()

        self.assertEqual(cockpit.ping_count_at_version, 3)


class HelmMotionCommandTests(unittest.TestCase):
    def setUp(self):
        temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(temp_dir.cleanup)
        self.helm = Helm(StartupCockpit(), temp_dir.name + "/events.log")

    def command(self, text):
        with contextlib.redirect_stdout(io.StringIO()):
            self.helm.command(text)

    def test_speed_selects_positive_magnitude_without_starting(self):
        self.assertEqual(self.helm._speed, 0.1)
        self.command("speed 0.2")
        self.assertEqual(self.helm._speed, 0.2)
        self.assertEqual(self.helm._direction, 0)
        self.assertFalse(self.helm._engaged)

    def test_direction_speed_change_and_stop(self):
        self.command("f")
        self.assertEqual(self.helm._direction, 1)
        self.assertEqual(self.helm._linear_speed(), 0.1)
        self.assertTrue(self.helm._engaged)

        self.command("speed 0.3")
        self.assertEqual(self.helm._speed, 0.3)
        self.assertEqual(self.helm._direction, 1)
        self.assertEqual(self.helm._linear_speed(), 0.3)

        self.helm._bank_dps = 20.0
        self.command("s")
        self.assertEqual(self.helm._direction, 0)
        self.assertEqual(self.helm._linear_speed(), 0.0)
        self.assertEqual(self.helm._bank_dps, 0.0)
        self.assertEqual(self.helm._speed, 0.3)

        self.command("b")
        self.assertEqual(self.helm._direction, -1)
        self.assertEqual(self.helm._linear_speed(), -0.3)

    def test_speed_rejects_zero_and_negative_values(self):
        self.command("speed 0")
        self.assertEqual(self.helm._speed, 0.1)
        self.command("speed -0.2")
        self.assertEqual(self.helm._speed, 0.1)

    def test_old_motion_commands_are_removed(self):
        for command in ("full", "half", "slow", "back full", "stop"):
            with self.subTest(command=command):
                output = io.StringIO()
                with contextlib.redirect_stdout(output):
                    self.helm.command(command)
                self.assertIn("unknown command", output.getvalue())


if __name__ == "__main__":
    unittest.main()