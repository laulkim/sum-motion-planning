import math
from pathlib import Path

import numpy as np

from simp_planner_tools.track_map import TrackMap, generate_clothoid_stadium


def test_clothoid_stadium_is_closed_and_g2_continuous():
    track = generate_clothoid_stadium()

    assert len(track.x) > 700
    assert abs(track.total_length - (80.0 + 2.0 * math.pi * 10.0 + 12.0)) < 0.5
    assert np.max(np.abs(track.kappa)) <= 0.1 + 1.0e-12
    assert np.max(np.abs(np.diff(np.r_[track.kappa, track.kappa[0]]))) < 0.004
    assert np.min(track.segment_length) > 0.19
    assert np.max(track.segment_length) <= 0.201


def test_installed_csv_contains_authored_curvature():
    map_file = Path(__file__).resolve().parents[1] / "maps" / "stadium_track.csv"
    track = TrackMap.load_csv(map_file)

    assert np.any(np.isclose(track.kappa, 0.0))
    assert np.any(np.isclose(track.kappa, 0.1))
    assert np.any((track.kappa > 0.0) & (track.kappa < 0.1))
