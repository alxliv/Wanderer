"""Cockpit API exceptions.

Every command either returns its answer or raises. Pilot code can treat
CockpitTimeout / CockpitLinkError as "the cockpit channel is broken" (the
airframe is meanwhile falling back to a stop on its own) and CockpitNack as
"the airframe heard me and said no".
"""


class CockpitError(Exception):
    """Base class for everything raised by the cockpit API."""


class CockpitTimeout(CockpitError):
    """No answer within the command timeout.

    On a healthy link this should never happen — the airframe answers every
    command immediately. Treat it as a link failure, not a slow command.
    """


class CockpitNack(CockpitError):
    """The airframe refused the command (e.g. drive while not armed).

    `code` is a short machine-readable reason; `message` is human-readable.
    """

    def __init__(self, code: str, message: str = ""):
        super().__init__(f"{code}: {message}" if message else code)
        self.code = code
        self.message = message


class CockpitLinkError(CockpitError):
    """The transport is unusable (not open, closed underneath us, I/O error)."""
