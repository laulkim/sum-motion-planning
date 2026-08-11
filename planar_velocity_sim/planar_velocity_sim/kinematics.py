from __future__ import annotations

import math


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def integrate_body_velocity(
    x: float,
    y: float,
    yaw: float,
    vx: float,
    vy: float,
    yaw_rate: float,
    dt: float,
) -> tuple[float, float, float]:
    """Midpoint integration of a planar body-frame velocity command."""
    if dt < 0.0:
        raise ValueError("dt must be non-negative")
    yaw_mid = float(yaw) + 0.5 * float(yaw_rate) * float(dt)
    cos_yaw = math.cos(yaw_mid)
    sin_yaw = math.sin(yaw_mid)
    next_x = float(x) + (cos_yaw * float(vx) - sin_yaw * float(vy)) * float(dt)
    next_y = float(y) + (sin_yaw * float(vx) + cos_yaw * float(vy)) * float(dt)
    next_yaw = wrap_angle(float(yaw) + float(yaw_rate) * float(dt))
    return next_x, next_y, next_yaw
