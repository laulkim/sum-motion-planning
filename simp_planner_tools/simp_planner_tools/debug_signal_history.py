from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class SourceTimeAligner:
    """Align a source header timestamp to the node-relative time axis."""

    origin_stamp_ns: Optional[int] = None
    origin_elapsed_s: Optional[float] = None
    last_time_s: Optional[float] = None

    def align(
        self,
        stamp_sec: int,
        stamp_nanosec: int,
        receive_elapsed_s: float,
    ) -> float:
        stamp_ns = int(stamp_sec) * 1_000_000_000 + int(stamp_nanosec)
        if stamp_ns <= 0:
            value = float(receive_elapsed_s)
        else:
            if self.origin_stamp_ns is None:
                self.origin_stamp_ns = stamp_ns
                self.origin_elapsed_s = float(receive_elapsed_s)
            assert self.origin_elapsed_s is not None
            value = self.origin_elapsed_s + 1.0e-9 * (
                stamp_ns - self.origin_stamp_ns
            )
        if self.last_time_s is not None and value < self.last_time_s:
            value = max(float(receive_elapsed_s), self.last_time_s)
        self.last_time_s = value
        return value
