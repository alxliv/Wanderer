import contextlib
import io
import math
import tempfile
import unittest

from cockpit.errors import CockpitTimeout
from cockpit.api import Cockpit, TurnStarted
from cockpit.events import ProcedureFinished, TacticalState
from cockpit.sim import SimulatedCockpitLink
from helm.helm import Helm


class StartupCockpit:
    def __init__(self):
        self.ping_count = 0
        self.ping_count_at_version = None

    def on_event(self, handler):
        self.event_handler = handler

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

    def start_turn(self, angle_rad, linear_m_s):
        self.turn_request = (angle_rad, linear_m_s)
        self.event_handler(ProcedureFinished(name="turn", outcome="DONE"))
        return TurnStarted(linear_m_s=max(-0.2, min(0.2, linear_m_s)),
                           timeout_s=1.0)

    def abort(self):
        self.abort_count = getattr(self, "abort_count", 0) + 1


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
        self.helm._cockpit.on_event(self.helm._on_event)

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

    def test_turn_is_started_in_firmware_without_odometry_polling(self):
        self.helm._state = TacticalState.ACTIVE
        self.command("speed 0.3")
        self.command("f")
        self.command("turn 90")

        angle_rad, linear_m_s = self.helm._cockpit.turn_request
        self.assertAlmostEqual(angle_rad, math.pi / 2)
        self.assertAlmostEqual(linear_m_s, 0.3)
        self.assertEqual(self.helm._speed, 0.2)
        self.assertTrue(self.helm._engaged)


class HelmTurnIntegrationTests(unittest.TestCase):
    def test_turn_waits_for_firmware_and_continues_straight(self):
        cockpit = Cockpit(SimulatedCockpitLink(), command_timeout=0.25)
        with tempfile.TemporaryDirectory() as temp_dir:
            helm = Helm(cockpit, temp_dir + "/events.log")
            with cockpit, contextlib.redirect_stdout(io.StringIO()):
                helm.start()
                helm.command("arm")
                helm.command("f")
                helm.command("turn 6")
                odometry = cockpit.odometry()
                helm.shutdown()

        self.assertAlmostEqual(odometry.left_m_s, 0.1)
        self.assertAlmostEqual(odometry.right_m_s, 0.1)


if __name__ == "__main__":
    unittest.main()