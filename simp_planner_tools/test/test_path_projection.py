import math

import numpy as np

from simp_planner_tools.path_geometry import project_closed_path
from simp_planner_tools.track_map import generate_clothoid_stadium


def test_continuous_projection_recovers_signed_lateral_offset():
    track = generate_clothoid_stadium()
    segment = int(np.argmax(track.kappa >= 0.1 - 1.0e-12)) + 20
    next_index = (segment + 1) % len(track.x)
    ratio = 0.37
    yaw_delta = math.atan2(
        math.sin(track.yaw[next_index] - track.yaw[segment]),
        math.cos(track.yaw[next_index] - track.yaw[segment]),
    )
    reference_yaw = track.yaw[segment] + ratio * yaw_delta
    reference_x = track.x[segment] + ratio * (
        track.x[next_index] - track.x[segment]
    )
    reference_y = track.y[segment] + ratio * (
        track.y[next_index] - track.y[segment]
    )
    offset = 0.35
    px = reference_x - offset * math.sin(reference_yaw)
    py = reference_y + offset * math.cos(reference_yaw)

    projection = project_closed_path(
        px,
        py,
        x=track.x,
        y=track.y,
        yaw=track.yaw,
        kappa=track.kappa,
        s=track.s,
        segment_length=track.segment_length,
        total_length=track.total_length,
    )

    assert projection.segment_index == segment
    assert abs(projection.t - ratio) < 0.02
    assert abs(projection.lateral_error - offset) < 1.0e-3
    assert abs(projection.kappa - 0.1) < 1.0e-10
