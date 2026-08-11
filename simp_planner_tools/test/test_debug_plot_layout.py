from __future__ import annotations

import matplotlib.pyplot as plt

from simp_planner_tools.debug_plot_layout import (
    create_debug_figure,
    first_true_time,
    symmetric_limit,
)


def test_debug_dashboard_layout_has_non_overlapping_primary_axes() -> None:
    figure, axes = create_debug_figure()
    figure.canvas.draw()

    map_box = axes.map.get_position()
    status_box = axes.status.get_position()
    speed_box = axes.speed.get_position()
    allocation_velocity_box = axes.allocation_velocity.get_position()
    allocation_consistency_box = axes.allocation_consistency.get_position()

    assert map_box.x1 < status_box.x0
    assert map_box.width > 2.5 * speed_box.width
    assert map_box.height > 0.85 * speed_box.height
    assert allocation_velocity_box.y0 > allocation_consistency_box.y1
    assert allocation_consistency_box.width > 3.5 * speed_box.width
    plt.close(figure)


def test_plot_scale_helpers() -> None:
    assert symmetric_limit([], minimum=2.0) == 2.0
    assert symmetric_limit([-3.0, 1.0], minimum=1.0) > 3.0
    assert first_true_time([0.0, 1.0, 2.0], [False, True, True]) == 1.0
    assert first_true_time([0.0], [False]) is None
