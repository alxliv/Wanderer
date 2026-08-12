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


if __name__ == "__main__":
    unittest.main()