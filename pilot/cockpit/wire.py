"""Pilot side of the cockpit wire (protocol/cockpit_protocol.md sections 2-4).

Formats request lines and parses airframe (downlink) lines. Pure text, no
transport: the future UartCockpitLink feeds serial lines through here. Kept
in lockstep with the firmware codec by the shared golden vectors in
protocol/cockpit_vectors.txt.
"""

from dataclasses import dataclass
from typing import Dict, Optional

MAX_LINE = 120  # spec section 2, including the terminator


def format_request(op: str, params: Optional[Dict[str, float]] = None) -> str:
    """Wire line for a request, without terminator.

    Velocities are formatted to three decimals -- the canonical form the
    golden vectors pin down.
    """
    if op == "drive":
        return "drive {:.3f} {:.3f}".format(
            params["linear_m_s"], params["angular_rad_s"])
    if params:
        raise ValueError(f"op {op!r} takes no params on the wire")
    return op


@dataclass
class Downlink:
    """One parsed airframe line.

    kind: "ok" | "err" | "state" | "fault" | "log" | "relay" | "skip"
      ok:    verb, fields ({key: value strings})
      err:   verb, reason, detail
      state: from_state, to_state
      fault: code (name string)
      log:   text
      relay: payload (verbatim)
      skip:  unknown sigil or blank -- ignore per spec section 2
    """
    kind: str
    verb: str = ""
    fields: Optional[Dict[str, str]] = None
    reason: str = ""
    detail: str = ""
    from_state: str = ""
    to_state: str = ""
    code: str = ""
    text: str = ""
    payload: str = ""


def _kv(tokens) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for tok in tokens:
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
        # Unknown/keyless tokens are ignored, per the spec's skip rules.
    return out


def parse_downlink(line: str) -> Downlink:
    line = line.rstrip("\r\n")
    if not line.strip():
        return Downlink("skip")
    sigil, rest = line[0], line[1:]

    if sigil == "=":
        tokens = rest.split()
        if len(tokens) >= 2 and tokens[0] == "ok":
            return Downlink("ok", verb=tokens[1], fields=_kv(tokens[2:]))
        if len(tokens) >= 3 and tokens[0] == "err":
            return Downlink("err", verb=tokens[1], reason=tokens[2],
                            detail=" ".join(tokens[3:]))
        return Downlink("skip")
    if sigil == "!":
        tokens = rest.split()
        if tokens and tokens[0] == "state":
            kv = _kv(tokens[1:])
            if "from" in kv and "to" in kv:
                return Downlink("state", from_state=kv["from"],
                                to_state=kv["to"])
        if tokens and tokens[0] == "fault":
            kv = _kv(tokens[1:])
            if "code" in kv:
                return Downlink("fault", code=kv["code"])
        return Downlink("skip")
    if sigil == "*":
        return Downlink("log", text=rest)
    if sigil == "^":
        return Downlink("relay", payload=rest)
    return Downlink("skip")
