import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from simp_planner_msgs.msg import TrackingTrajectory, TrackingTrajectoryPoint

from simp_planner_tools.tracking_shutdown_plot import COLUMNS, create_figures, sample_row


def test_shutdown_figures_use_timestamp_reference_and_real_command(tmp_path):
    trajectory = TrackingTrajectory()
    trajectory.header.frame_id = "odom"
    trajectory.header.stamp.sec = 1
    trajectory.sample_period.nanosec = 10000000
    trajectory.points = [TrackingTrajectoryPoint(), TrackingTrajectoryPoint(x=1.0, y=2.0, vx=0.5)]
    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.header.stamp.sec = 1
    odom.header.stamp.nanosec = 1
    odom.pose.pose.orientation.w = 1.0
    command = Twist()
    command.linear.x = 0.7
    row = sample_row(odom, trajectory, command, 1000000000)
    assert len(row) == len(COLUMNS)
    assert row[4:10] == (1.0, 2.0, 0.0, 1.0, 2.0, 0.0)
    assert row[13] == 0.5 and row[16] == 0.7
    figures = create_figures(np.asarray([row, row]))
    assert [len(figure.axes) for figure in figures] == [3, 6]
    np.testing.assert_allclose(figures[1].axes[1].lines[0].get_ydata(), [0.7, 0.7])
    for index, figure in enumerate(figures):
        figure.savefig(tmp_path / f"figure{index}.png")
        plt.close(figure)
    odom.header.stamp.nanosec = 10000001
    assert math.isnan(sample_row(odom, trajectory, command, 1000000000)[7])
    odom.header.stamp.nanosec = 0
    odom.header.frame_id = "map"
    assert math.isnan(sample_row(odom, trajectory, command, 1000000000)[7])
