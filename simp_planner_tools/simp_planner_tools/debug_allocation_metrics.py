from __future__ import annotations

"""Direct algebraic diagnostics for Motion-Body Allocation outputs.

This module intentionally contains no numerical differentiation.  It converts
existing allocator outputs to display units and evaluates exact identities.
"""

from dataclasses import dataclass
import math


MODE_BETA_CENTER_DEG = {0: 0.0, 1: 180.0, 2: 90.0, 3: -90.0}


def wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class AllocationDebugSample:
    motion_heading_rate_degps: float
    motion_heading_acceleration_degps2: float
    beta_deg: float
    beta_center_deg: float
    beta_deviation_deg: float
    beta_rate_degps: float
    beta_acceleration_degps2: float
    yaw_acceleration_degps2: float
    rate_split_residual_degps: float
    speed_reconstruction_error_mps: float


def compute_allocation_debug_sample(
    *,
    vx: float,
    vy: float,
    yaw_rate: float,
    planned_speed: float,
    motion_heading_rate: float,
    motion_heading_acceleration: float,
    beta: float,
    beta_rate: float,
    yaw_acceleration: float,
    mode: int | None,
) -> AllocationDebugSample:
    values = [
        vx,
        vy,
        yaw_rate,
        planned_speed,
        motion_heading_rate,
        motion_heading_acceleration,
        beta,
        beta_rate,
        yaw_acceleration,
    ]
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("allocation debug inputs must be finite")

    motion_rate_deg = math.degrees(float(motion_heading_rate))
    motion_accel_deg = math.degrees(float(motion_heading_acceleration))
    beta_deg = math.degrees(float(beta))
    beta_rate_deg = math.degrees(float(beta_rate))
    yaw_rate_deg = math.degrees(float(yaw_rate))
    yaw_accel_deg = math.degrees(float(yaw_acceleration))
    beta_accel_deg = motion_accel_deg - yaw_accel_deg

    center = math.nan
    deviation = math.nan
    if mode is not None and int(mode) in MODE_BETA_CENTER_DEG:
        center = float(MODE_BETA_CENTER_DEG[int(mode)])
        deviation = math.degrees(
            wrap_angle(float(beta) - math.radians(center))
        )

    return AllocationDebugSample(
        motion_heading_rate_degps=motion_rate_deg,
        motion_heading_acceleration_degps2=motion_accel_deg,
        beta_deg=beta_deg,
        beta_center_deg=center,
        beta_deviation_deg=deviation,
        beta_rate_degps=beta_rate_deg,
        beta_acceleration_degps2=beta_accel_deg,
        yaw_acceleration_degps2=yaw_accel_deg,
        rate_split_residual_degps=(
            motion_rate_deg - yaw_rate_deg - beta_rate_deg
        ),
        speed_reconstruction_error_mps=(
            math.hypot(float(vx), float(vy)) - float(planned_speed)
        ),
    )
