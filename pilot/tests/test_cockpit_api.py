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


class SpecMatrixTest(unittest.TestCase):
    """Cockpit spec section 6, one test per cell.

    Rows are requests, columns are the state the request arrives in; each
    cell is one normative outcome (transition + ok, ok no-op, or a nack with
    a specific reason). This table is a transcription of the spec -- the same
    cells as firmware/common/test_tactical.cpp -- and must never be "fixed"
    to match code: when they disagree, the spec wins.

    The suite runs against the sim today and, unchanged, against the real
    firmware once a UART CockpitLink exists: nothing here knows the transport.
    """

    S = TacticalState
    #        op             in state     expected state  nack code (None = ok)
    MATRIX = [
        ("arm",         S.SAFE,     S.ACTIVE,   None),
        ("arm",         S.ACTIVE,   S.ACTIVE,   None),
        ("arm",         S.FALLBACK, S.FALLBACK, None),   # arm never exits FALLBACK
        ("arm",         S.FAULT,    S.FAULT,    "fault_latched"),
        ("disarm",      S.SAFE,     S.SAFE,     None),
        ("disarm",      S.ACTIVE,   S.SAFE,     None),
        ("disarm",      S.FALLBACK, S.SAFE,     None),   # the "stand down"
        ("disarm",      S.FAULT,    S.FAULT,    "fault_latched"),
        ("drive",       S.SAFE,     S.SAFE,     "not_armed"),
        ("drive",       S.ACTIVE,   S.ACTIVE,   None),
        ("drive",       S.FALLBACK, S.ACTIVE,   None),   # the ONLY resume
        ("drive",       S.FAULT,    S.FAULT,    "not_armed"),
        ("stop",        S.SAFE,     S.SAFE,     "not_armed"),
        ("stop",        S.ACTIVE,   S.ACTIVE,   None),
        ("stop",        S.FALLBACK, S.FALLBACK, "not_armed"),  # refusal tells truth
        ("stop",        S.FAULT,    S.FAULT,    "not_armed"),
        ("estop",       S.SAFE,     S.FAULT,    None),
        ("estop",       S.ACTIVE,   S.FAULT,    None),
        ("estop",       S.FALLBACK, S.FAULT,    None),
        ("estop",       S.FAULT,    S.FAULT,    None),   # idempotent, no 2nd event
        ("clear_fault", S.SAFE,     S.SAFE,     "no_fault"),
        ("clear_fault", S.ACTIVE,   S.ACTIVE,   "no_fault"),
        ("clear_fault", S.FALLBACK, S.FALLBACK, "no_fault"),
        ("clear_fault", S.FAULT,    S.SAFE,     None),
    ]
    # 25th cell, clear_fault -> fault_persists: see test_fault_persists below.

    def _fresh(self):
        """A brand-new sim + cockpit, the Python tac_init()."""
        cockpit = Cockpit(SimulatedCockpitLink(liveness_timeout_s=LIVENESS_S))
        events: "queue.Queue" = queue.Queue()
        cockpit.on_event(events.put)
        cockpit.open()
        return cockpit, events

    def _enter(self, cockpit, events, target):
        if target is TacticalState.SAFE:
            return
        cockpit.arm()
        if target is TacticalState.ACTIVE:
            return
        if target is TacticalState.FALLBACK:
            cockpit.drive(0.2, 0.0)
            wait_for(events, lambda e: isinstance(e, StateChanged)
                     and e.new == TacticalState.FALLBACK)
            return
        cockpit.estop()  # -> FAULT

    def _drain(self, events):
        drained = []
        try:
            while True:
                drained.append(events.get_nowait())
        except queue.Empty:
            return drained

    def test_spec_section6_matrix(self):
        ops = {
            "arm": lambda c: c.arm(),
            "disarm": lambda c: c.disarm(),
            "stop": lambda c: c.stop(),
            "drive": lambda c: c.drive(0.2, 0.0),
            "estop": lambda c: c.estop(),
            "clear_fault": lambda c: c.clear_fault(),
        }
        for op, in_state, out_state, nack in self.MATRIX:
            with self.subTest(op=op, state=in_state.name):
                cockpit, events = self._fresh()
                try:
                    self._enter(cockpit, events, in_state)
                    time.sleep(0.05)   # let async event dispatch settle
                    self._drain(events)

                    if nack is None:
                        ops[op](cockpit)
                    else:
                        with self.assertRaises(CockpitNack) as ctx:
                            ops[op](cockpit)
                        self.assertEqual(ctx.exception.code, nack)

                    self.assertEqual(cockpit.state(), out_state)

                    if out_state != in_state:
                        # A transition must be reported.
                        wait_for(events, lambda e: isinstance(e, StateChanged)
                                 and e.new == out_state)
                    else:
                        # No transition: nothing may be reported.
                        time.sleep(0.05)
                        leftovers = [e for e in self._drain(events)
                                     if isinstance(e, (StateChanged, FaultRaised))]
                        self.assertEqual(leftovers, [],
                                         f"{op} in {in_state.name} emitted {leftovers}")
                finally:
                    cockpit.close()

    def test_fault_before_state_on_latch(self):
        """On a fault latch, FaultRaised is delivered before StateChanged --
        mirroring the firmware's `!fault` then `!state` line order."""
        cockpit, events = self._fresh()
        try:
            cockpit.arm()
            time.sleep(0.05)
            self._drain(events)
            cockpit.estop()
            first = events.get(timeout=2.0)
            self.assertIsInstance(first, FaultRaised)
            self.assertEqual(first.code, 1)  # ESTOP
            second = events.get(timeout=2.0)
            self.assertIsInstance(second, StateChanged)
            self.assertEqual(second.new, TacticalState.FAULT)
        finally:
            cockpit.close()

    @unittest.skip("sim models no persistent fault yet -- cockpit spec "
                   "section 11; enable with the first Tier 3 fault")
    def test_fault_persists(self):
        """clear_fault while the fault condition is still present must nack
        fault_persists and keep the latch (spec section 6, FAULT column)."""


if __name__ == "__main__":
    unittest.main()
