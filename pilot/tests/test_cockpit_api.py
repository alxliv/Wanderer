"""Cockpit API contract tests, run against the simulated airframe.

These pin down the milestone-one semantics ("Pi5 drives the rover over UART
with the deadman working") that the real UART link + firmware must later
satisfy: blocking commands, nacks, callbacks, and the fallback/resume rule.

Run from pilot/:  python -m unittest discover -s tests -v
"""

import queue
import time
import unittest

from cockpit import (Cockpit, CockpitNack, FaultRaised, StateChanged,
                     TacticalState)
from cockpit.sim import SimulatedCockpitLink

LIVENESS_S = 0.15  # short deadman so tests run fast


def wait_for(events: "queue.Queue", predicate, timeout=2.0):
    """Pop events until one satisfies predicate; fail on timeout."""
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise AssertionError("expected event did not arrive")
        event = events.get(timeout=remaining)
        if predicate(event):
            return event


class CockpitApiTest(unittest.TestCase):
    def setUp(self):
        self.cockpit = Cockpit(
            SimulatedCockpitLink(liveness_timeout_s=LIVENESS_S))
        self.events: "queue.Queue" = queue.Queue()
        self.cockpit.on_event(self.events.put)
        self.cockpit.open()

    def tearDown(self):
        self.cockpit.close()

    def test_arm_drive_and_query(self):
        self.assertEqual(self.cockpit.state(), TacticalState.SAFE)
        self.cockpit.arm()
        self.assertEqual(self.cockpit.state(), TacticalState.ACTIVE)
        self.cockpit.drive(0.3, 0.0)
        time.sleep(0.1)  # let the sim integrate a little motion
        odo = self.cockpit.odometry()
        self.assertGreater(odo.left_ticks, 0)
        self.assertAlmostEqual(odo.left_m_s, 0.3, places=3)

    def test_drive_unarmed_is_nacked(self):
        with self.assertRaises(CockpitNack) as ctx:
            self.cockpit.drive(0.2, 0.0)
        self.assertEqual(ctx.exception.code, "not_armed")

    def test_deadman_falls_back_and_resumes_only_on_drive(self):
        self.cockpit.arm()
        self.cockpit.drive(0.3, 0.0)

        # Pilot goes quiet -> airframe zombies out on its own and says so.
        wait_for(self.events, lambda e: isinstance(e, StateChanged)
                 and e.new == TacticalState.FALLBACK)

        # Queries feed liveness but must NOT resume motion.
        self.assertEqual(self.cockpit.state(), TacticalState.FALLBACK)
        time.sleep(0.05)
        self.assertEqual(self.cockpit.state(), TacticalState.FALLBACK)

        # Only a fresh drive re-asserts intent and resumes.
        self.cockpit.drive(0.2, 0.0)
        self.assertEqual(self.cockpit.state(), TacticalState.ACTIVE)

    def test_estop_latches_until_cleared(self):
        self.cockpit.arm()
        self.cockpit.estop()
        wait_for(self.events, lambda e: isinstance(e, FaultRaised))
        self.assertEqual(self.cockpit.state(), TacticalState.FAULT)

        with self.assertRaises(CockpitNack):
            self.cockpit.drive(0.1, 0.0)
        with self.assertRaises(CockpitNack):  # disarm refused while latched
            self.cockpit.disarm()

        self.cockpit.clear_fault()
        self.assertEqual(self.cockpit.state(), TacticalState.SAFE)

    def test_stop_zeroes_velocity_but_stays_active(self):
        self.cockpit.arm()
        self.cockpit.drive(0.3, 0.0)
        self.cockpit.stop()
        self.assertEqual(self.cockpit.state(), TacticalState.ACTIVE)
        odo = self.cockpit.odometry()
        self.assertEqual(odo.left_m_s, 0.0)


if __name__ == "__main__":
    unittest.main()
