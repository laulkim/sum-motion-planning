"""새 토픽 기반 대시보드: reference, 실제 command, nominal 진단을 분리한다."""
import copy
import json
import math

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from simp_planner_msgs.msg import (
    ReferencePath, TrackingTrajectory, TrackingTrajectoryPoint,
    TrackingDiagnostics, TrackingDiagnosticsPoint,
)
from simp_planner_tools.debug_plot_node import DebugPlotNode


@pytest.fixture
def node(tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'output_dir:={tmp_path}', '-p', 'save_period:=1000.0'])
    result = DebugPlotNode()
    yield result
    result.destroy_node()
    rclpy.shutdown()


def trajectory(node, cls, points):
    msg = cls()
    msg.header.frame_id = 'odom'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.sample_period.nanosec = 10000000
    msg.points = points
    return msg


def test_reference_command_and_diagnostics_are_independent(node):
    ref = trajectory(node, TrackingTrajectory, [TrackingTrajectoryPoint(x=1., y=2., vx=3.)] * 31)
    debug = trajectory(node, TrackingDiagnostics, [TrackingDiagnosticsPoint(
        vx=3., planned_speed=3., planned_acceleration=.4, planned_jerk=.5,
        allocation_valid=True, motion_curvature=.1, motion_heading_rate=.3,
        yaw_rate=.2, beta_rate=.1)] * 31)
    debug.header = ref.header
    node.tracking_callback(ref)
    node.diagnostics_callback(debug)
    old = trajectory(node, TrackingTrajectory, [TrackingTrajectoryPoint(x=99.)])
    old.header.stamp.sec = ref.header.stamp.sec - 1
    node.tracking_callback(old)
    assert node.latest_trajectory is ref
    cmd = Twist()
    cmd.linear.x = 3.5
    node.command_callback(cmd)
    path = ReferencePath(x=[0., 10., 20.], y=[0., 0., 0.], yaw=[0., 0., 0.])
    node.reference_callback(path)
    odom = Odometry()
    odom.header = copy.deepcopy(ref.header)
    odom.pose.pose.orientation.w = 1.
    odom.twist.twist.linear.x = 3.4
    node.odom_callback(odom)
    row = node.tracking_rows[-1]
    assert row[8] == 2. and row[10] == 3.4 and row[13] == 3. and row[16] == 3.5
    assert node.cmd_vx == 3.5  # nominal 진단이 실제 command를 덮어쓰지 않는다.
    assert node.command_acceleration_history == [.4]
    assert node.command_jerk_history == [.5]
    assert node.executed_kappa_history == [.1]
    assert node.allocation_rate_split_residual_history[-1] == pytest.approx(0.)
    odom.header.frame_id = 'map'
    node.odom_callback(odom)
    assert math.isnan(node.tracking_rows[-1][8])


def test_unavailable_allocation_is_nan_and_missing_input_is_not_ok(node):
    node.record_diagnostics(TrackingDiagnosticsPoint(allocation_valid=False), 0.)
    assert math.isnan(node.command_beta_history[-1])
    for name in node.last_seen:
        node.mark(name)
    node.latest_trajectory = None
    node.last_seen['tracking_trajectory'] = None
    assert node.diagnose()[0] == 'NO_DATA'
    node.mark('tracking_trajectory')
    assert node.diagnose()[0] == 'N/A'


def test_rich_plan_status_survives_same_plan_short_status(node):
    rich = {'execution': {'current_plan_id': 1}, 'plan': {
        'id': 1, 'selected_n_target': .25, 'curvature_switch_jump': None,
        'total_compute_time_ms': 5.}}
    node.planner_status_callback(String(data=json.dumps(rich)))
    node.planner_status_callback(String(data=json.dumps({
        'execution': {'current_plan_id': 1}, 'plan': {'total_compute_time_ms': 6.}})))
    assert node.planner_section('plan')['selected_n_target'] == .25
    assert node.n_target_history == [.25]
    assert math.isnan(node.curvature_jump_history[0])
    node.planner_status_callback(String(data=json.dumps({
        'execution': {'current_plan_id': 2}, 'plan': {'total_compute_time_ms': 7.}})))
    assert 'selected_n_target' not in node.planner_section('plan')
