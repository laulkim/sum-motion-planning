import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from simp_planner_msgs.msg import TrackingControl, TrackingTrajectoryPoint

from simp_planner_tools.tracking_shutdown_plot import COLUMNS, create_figures, sample_row


def test_shutdown_figures_use_controller_snapshot(tmp_path):
    message = TrackingControl(valid=True)
    message.header.stamp.sec = 1
    message.header.stamp.nanosec = 14000000
    message.odom_stamp.sec = 1
    message.reference_stamp.sec = 1
    message.reference_stamp.nanosec = 20000000
    message.predicted_x = 0.04
    message.reference = TrackingTrajectoryPoint(x=0.04, vx=2.0)
    message.measured_velocity.linear.x = 2.0
    message.command.linear.x = 2.0
    row = sample_row(message, 1000000000)
    assert len(row) == len(COLUMNS)
    assert math.isclose(row[0], 0.014)
    assert row[1:10] == (0.04, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert row[10] == row[13] == row[16] == 2.0
    assert row[19:] == (1014000000, 1000000000, 1020000000, 1)
    # Keep the controller's error verbatim; never select/recompute it in the plotter.
    message.ex, message.ey, message.etheta = 0.1, -0.2, 0.3
    assert sample_row(message, 1000000000)[7:10] == (0.1, -0.2, 0.3)
    message.valid = False
    message.command.linear.x = 0.0
    invalid = sample_row(message, 1000000000)
    assert all(math.isnan(value) for value in invalid[1:16])
    assert invalid[16:19] == (0.0, 0.0, 0.0)
    assert invalid[-1] == 0
    figures = create_figures(np.asarray([row, invalid]))
    assert [len(figure.axes) for figure in figures] == [3, 6]
    np.testing.assert_allclose(figures[0].axes[0].lines[0].get_ydata(), [0.0, np.nan])
    np.testing.assert_allclose(figures[1].axes[1].lines[0].get_ydata(), [2.0, 0.0])
    assert figures[1].axes[0].lines[0].get_label() == "predicted"
    for index, figure in enumerate(figures):
        figure.savefig(tmp_path / f"figure{index}.png")
        plt.close(figure)
    # Old CSVs remain identifiable as the old, unpredicted calculation.
    legacy = create_figures(np.asarray([row[:19]]), predicted=False)
    assert legacy[1].axes[0].lines[0].get_label() == "actual"
    for figure in legacy:
        plt.close(figure)
