"""wire.py against the shared golden vectors (protocol/cockpit_vectors.txt).

The pilot exercises the [request] section (its formatting direction) and the
[downlink] section (its parsing direction); the firmware codec test covers
the complementary [parse] and [format] sections of the same file. A vector
change fails whichever side disagrees -- that is the point.
"""

import pathlib
import unittest

from cockpit import wire

VECTORS = (pathlib.Path(__file__).resolve().parents[2]
           / "protocol" / "cockpit_vectors.txt")


def load_sections():
    sections = {}
    current = None
    for raw in VECTORS.read_text().splitlines():
        if raw.startswith("#"):
            continue
        if raw.startswith("["):
            current = raw.strip()
            sections[current] = []
            continue
        if not raw.strip():
            continue
        sections[current].append(raw.split("|"))
    return sections


class WireVectorsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sections = load_sections()

    def test_vectors_file_present_and_complete(self):
        self.assertIn("[request]", self.sections)
        self.assertIn("[downlink]", self.sections)
        self.assertGreater(len(self.sections["[request]"]), 5)
        self.assertGreater(len(self.sections["[downlink]"]), 5)

    def test_request_formatting(self):
        for fields in self.sections["[request]"]:
            with self.subTest(vector=fields):
                if fields[0] == "drive":
                    op, lin, ang, expected = fields
                    got = wire.format_request(op, {
                        "linear_m_s": float(lin),
                        "angular_rad_s": float(ang)})
                else:
                    op, expected = fields
                    got = wire.format_request(op)
                self.assertEqual(got, expected)
                self.assertLess(len(got) + 2, wire.MAX_LINE)

    def test_downlink_parsing(self):
        for fields in self.sections["[downlink]"]:
            with self.subTest(vector=fields):
                line, kind, rest = fields[0], fields[1], fields[2:]
                d = wire.parse_downlink(line)
                self.assertEqual(d.kind, kind)
                if kind == "ok":
                    self.assertEqual(d.verb, rest[0])
                    expected_fields = dict(
                        tok.split("=", 1) for tok in rest[1].split() if tok)
                    self.assertEqual(d.fields, expected_fields)
                elif kind == "err":
                    self.assertEqual(d.verb, rest[0])
                    self.assertEqual(d.reason, rest[1])
                    self.assertEqual(d.detail, rest[2])
                elif kind == "state":
                    self.assertEqual((d.from_state, d.to_state),
                                     (rest[0], rest[1]))
                elif kind == "fault":
                    self.assertEqual(d.code, rest[0])
                elif kind == "log":
                    self.assertEqual(d.text, rest[0])
                elif kind == "relay":
                    self.assertEqual(d.payload, rest[0])

    def test_terminator_tolerance(self):
        # Real serial lines arrive with terminators; parsing must not care.
        self.assertEqual(wire.parse_downlink("=ok ping\r\n").kind, "ok")
        self.assertEqual(wire.parse_downlink("=ok ping\n").kind, "ok")


if __name__ == "__main__":
    unittest.main()
