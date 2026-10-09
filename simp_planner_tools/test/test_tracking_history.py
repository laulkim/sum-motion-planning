import pytest

from simp_planner_tools.tracking_history import TrackingPlayback

PERIOD = 10_000_000


def test_each_trajectory_governs_until_the_next_stamp():
    playback = TrackingPlayback(PERIOD)
    assert playback.add(0, 100, "a") == []
    emitted = playback.add(100_000_000, 100, "b")
    assert [(p, i, t) for p, i, t in emitted] == [("a", i, i * PERIOD) for i in range(10)]
    emitted = playback.add(200_000_000, 100, "c")
    assert [p for p, _, _ in emitted] == ["b"] * 10
    assert [i for _, i, _ in emitted] == list(range(10))
    assert emitted[0][2] == 100_000_000


def test_past_the_end_the_last_point_is_held():
    playback = TrackingPlayback(PERIOD)
    playback.add(0, 3, "short")
    emitted = playback.flush(60_000_000)
    assert [i for _, i, _ in emitted] == [0, 1, 2, 2, 2, 2]


def test_flush_then_newer_trajectory_does_not_repeat_points():
    playback = TrackingPlayback(PERIOD)
    playback.add(0, 100, "a")
    first = playback.flush(150_000_000)
    assert [t for _, _, t in first] == [i * PERIOD for i in range(15)]
    # The newer trajectory is stamped 0.1 s but arrived after the flush; its
    # points before 0.15 s repeat what was already emitted from "a".
    assert playback.add(100_000_000, 100, "b") == []
    emitted = playback.flush(200_000_000)
    assert [(p, i, t) for p, i, t in emitted] == [
        ("b", i, 100_000_000 + i * PERIOD) for i in range(5, 10)
    ]


def test_out_of_order_trajectory_is_ignored():
    playback = TrackingPlayback(PERIOD)
    playback.add(100_000_000, 10, "new")
    assert playback.add(0, 10, "old") == []
    emitted = playback.flush(120_000_000)
    assert [p for p, _, _ in emitted] == ["new", "new"]


def test_invalid_inputs_are_rejected():
    with pytest.raises(ValueError):
        TrackingPlayback(0)
    with pytest.raises(ValueError):
        TrackingPlayback(PERIOD).add(0, 0, "empty")
