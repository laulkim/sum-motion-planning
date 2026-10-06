from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class TrackingPlayback:
    """Rebuild which tracking-trajectory points the controller sent, and when.

    The controller always plays the newest trajectory and, within it, the
    point whose time (stamp + index * period) is not after now; past the end
    it holds the last point. Each trajectory therefore governs from its stamp
    until the next trajectory's stamp. Points are emitted once each, in time
    order, as (payload, index, time_ns).
    """

    period_ns: int
    _latest: Optional[tuple[int, int, Any]] = field(default=None, init=False)
    _next_ns: Optional[int] = field(default=None, init=False)

    def __post_init__(self) -> None:
        if self.period_ns <= 0:
            raise ValueError("period_ns must be positive")

    def add(self, stamp_ns: int, size: int, payload: Any) -> list[tuple[Any, int, int]]:
        """Register a trajectory; return the previous one's points that ran before it."""
        if size <= 0:
            raise ValueError("tracking trajectory must not be empty")
        if self._latest is not None and stamp_ns < self._latest[0]:
            return []  # Out-of-order delivery never replaces a newer trajectory.
        emitted = self.flush(stamp_ns)
        self._latest = (int(stamp_ns), int(size), payload)
        return emitted

    def flush(self, until_ns: int) -> list[tuple[Any, int, int]]:
        """Return the newest trajectory's points that ran before until_ns."""
        if self._latest is None:
            return []
        stamp_ns, size, payload = self._latest
        start_ns = stamp_ns if self._next_ns is None else max(stamp_ns, self._next_ns)
        # A newer trajectory repeats the old one before its own handover time,
        # so points already emitted from the old one are not emitted again.
        index = -(-(start_ns - stamp_ns) // self.period_ns)
        time_ns = stamp_ns + index * self.period_ns
        emitted = []
        while time_ns < until_ns:
            emitted.append((payload, min(index, size - 1), time_ns))
            index += 1
            time_ns += self.period_ns
        self._next_ns = time_ns
        return emitted
