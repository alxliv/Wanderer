"""The real UART transport behind the CockpitLink interface.

Wraps a serial port (e.g. /dev/serial0 on the Pi at 115200 8N1, spec
section 12) around the pure-text codec in wire.py: requests are formatted
and written with CRLF, downlink lines are parsed on a reader thread and
sorted into replies (handed to the blocked execute()), events (handed to
the API's sink), and log/relay lines (optional sinks, dropped by default).

Semantic decode lives here: the wire speaks short field names and state
NAMES (lt/rt, state=ACTIVE, fw=0.1); the API speaks long names and enums.
The simulator link speaks the API form directly, which is why this mapping
exists only on the real transport.

pyserial is imported lazily in open(), so sim-only setups need not have it.
"""

import threading
import time
from typing import Callable, Optional

from . import link as _link
from . import wire
from .errors import CockpitLinkError, CockpitNack, CockpitTimeout
from .events import (Event, FaultRaised, ProcedureFinished, StateChanged,
                     TacticalState)
from .link import CockpitLink, Reply, Request

# Wire fault names <-> the integer codes of events.FaultRaised (spec
# section 8). Grows with the Tier 3 registry. Unknown names map to -1
# rather than being dropped: a fault the pilot cannot name is still a fault.
FAULT_CODES = {"ESTOP": 1}
FAULT_NAMES = {v: k for k, v in FAULT_CODES.items()}


def _decode(op: str, d: wire.Downlink) -> Reply:
    """Wire fields of one `=ok` line -> the Reply values api.py expects."""
    f = d.fields or {}
    if op == _link.OP_GET_STATE:
        return Reply({"state": int(TacticalState[f["state"]])})
    if op == _link.OP_GET_ODOMETRY:
        return Reply({"left_ticks": int(f["lt"]), "right_ticks": int(f["rt"]),
                      "left_m_s": float(f["vl"]), "right_m_s": float(f["vr"])})
    if op == _link.OP_GET_HEADING:
        return Reply({"psi_rad": float(f["psi"]), "rate_rad_s": float(f["rate"]),
                      "bias_rad_s": float(f["bias"]), "valid": f["valid"] == "1"})
    if op == _link.OP_GET_VERSION:
        major, minor = f["fw"].split(".", 1)
        return Reply({"major": int(major), "minor": int(minor)})
    if op == _link.OP_GET_GEOMETRY:
        return Reply({"ticks_per_meter": float(f["tpm"]),
                      "track_m": float(f["track"])})
    if op == _link.OP_GET_MOTOR_CONFIG:
        return Reply({"left_gain_permille": int(f["lgain"]),
                      "right_gain_permille": int(f["rgain"]),
                      "left_deadband_permille": int(f["ldead"]),
                      "right_deadband_permille": int(f["rdead"])})
    if op == _link.OP_GET_MOVE_STATUS:
        return Reply({"active": f["a"] == "1",
                      "elapsed_s": int(f["t"]) / 1000.0,
                      "heading_ref_rad": int(f["h"]) / 1000.0,
                      "heading_rad": int(f["x"]) / 1000.0,
                      "error_rad": int(f["e"]) / 1000.0,
                      "rate_rad_s": int(f["v"]) / 1000.0,
                      "p_rad_s": int(f["p"]) / 1000.0,
                      "i_rad_s": int(f["i"]) / 1000.0,
                      "d_rad_s": int(f["d"]) / 1000.0,
                      "omega_rad_s": int(f["o"]) / 1000.0,
                      "left_m_s": int(f["l"]) / 1000.0,
                      "right_m_s": int(f["r"]) / 1000.0,
                      "saturation": int(f["s"])})
    if op == _link.OP_PROC:
        return Reply({"linear_m_s": float(f["lin"]),
                      "timeout_s": float(f["timeout"])})
    if op == _link.OP_DRIVE and "lin" in f:
        # Applied pair present only when the request was scaled (spec s3).
        return Reply({"linear_m_s": float(f["lin"]),
                      "angular_rad_s": float(f["omega"])})
    return Reply()


class UartCockpitLink(CockpitLink):
    def __init__(self, port: str, baud: int = 115200):
        self._port_name = port
        self._baud = baud
        self._serial = None
        self._reader: Optional[threading.Thread] = None
        self._running = False
        self._sink: Optional[Callable[[Event], None]] = None
        self._log_sink: Optional[Callable[[str], None]] = None
        self._relay_sink: Optional[Callable[[str], None]] = None
        # One request in flight (API guarantee): its reply crosses threads
        # through this one-slot mailbox.
        self._reply_cv = threading.Condition()
        self._reply: Optional[wire.Downlink] = None
        self._write_lock = threading.Lock()

    # ---- extra sinks (beyond the CockpitLink contract) -------------------

    def set_log_sink(self, sink: Callable[[str], None]) -> None:
        """Receives the text of `*` log lines (spec s2); default: dropped."""
        self._log_sink = sink

    def set_relay_sink(self, sink: Callable[[str], None]) -> None:
        """Receives `^` relay payloads verbatim; default: dropped."""
        self._relay_sink = sink

    # ---- CockpitLink -----------------------------------------------------

    def set_event_sink(self, sink: Callable[[Event], None]) -> None:
        self._sink = sink

    def open(self) -> None:
        try:
            import serial  # lazy: sim-only setups need not install pyserial
        except ImportError as e:
            raise CockpitLinkError(
                "pyserial is required for the UART link "
                "(pip install pyserial)") from e
        try:
            self._serial = serial.Serial(self._port_name, self._baud,
                                         timeout=0.05)
        except Exception as e:
            raise CockpitLinkError(f"cannot open {self._port_name}: {e}") from e
        self._running = True
        self._reader = threading.Thread(target=self._read_loop,
                                        name="cockpit-uart", daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._running = False
        if self._reader is not None:
            self._reader.join(timeout=1.0)
            self._reader = None
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def execute(self, request: Request, timeout: float) -> Reply:
        if self._serial is None:
            raise CockpitLinkError("link is not open")
        line = wire.format_request(request.op, dict(request.params))
        with self._reply_cv:
            self._reply = None  # anything older is stale by definition
            with self._write_lock:
                try:
                    self._serial.write(line.encode("ascii") + b"\r\n")
                except Exception as e:
                    raise CockpitLinkError(f"write failed: {e}") from e
            deadline = time.monotonic() + timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise CockpitTimeout(f"no reply to {request.op!r} "
                                         f"within {timeout} s")
                self._reply_cv.wait(remaining)
                d, self._reply = self._reply, None
                if d is None:
                    continue
                # Verb echo is the desync guard (spec s3): a mismatch means
                # a stale or lost line -- discard, keep waiting.
                if d.verb != request.op and d.verb != "?":
                    continue
                if d.kind == "err":
                    raise CockpitNack(d.reason, d.detail)
                return _decode(request.op, d)

    # ---- reader thread ---------------------------------------------------

    def _read_loop(self) -> None:
        buf = bytearray()
        prev_cr = False
        while self._running:
            try:
                data = self._serial.read(64)
            except Exception:
                return  # port died; pending execute() will time out
            for b in data:
                c = chr(b & 0x7F)
                if c == "\n":
                    if not prev_cr:      # bare LF terminates; LF after CR
                        self._line(buf)  # is the same terminator (spec s2)
                        buf.clear()
                    prev_cr = False
                elif c == "\r":
                    self._line(buf)
                    buf.clear()
                    prev_cr = True
                else:
                    prev_cr = False
                    if len(buf) < wire.MAX_LINE:
                        buf.append(ord(c))

    def _line(self, buf: bytearray) -> None:
        d = wire.parse_downlink(buf.decode("ascii", errors="replace"))
        if d.kind in ("ok", "err"):
            with self._reply_cv:
                self._reply = d
                self._reply_cv.notify()
        elif d.kind == "state" and self._sink is not None:
            try:
                self._sink(StateChanged(old=TacticalState[d.from_state],
                                        new=TacticalState[d.to_state]))
            except KeyError:
                pass  # unknown state name from newer firmware: skip (spec s2)
        elif d.kind == "fault" and self._sink is not None:
            self._sink(FaultRaised(code=FAULT_CODES.get(d.code, -1)))
        elif d.kind == "proc" and self._sink is not None:
            self._sink(ProcedureFinished(name=d.name, outcome=d.outcome,
                                         reason=d.reason))
        elif d.kind == "log" and self._log_sink is not None:
            self._log_sink(d.text)
        elif d.kind == "relay" and self._relay_sink is not None:
            self._relay_sink(d.payload)
        # "skip": per spec, silently ignored.
