#!/usr/bin/env python3
"""Regression: a completed crab leg -> boundary spot turn -> Forward READY.

Start at the completed crab leg to isolate mode/path callback ordering from
open-loop drift along the full course. Uses a private ROS domain and simulator.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--domain', type=int, default=165)
    parser.add_argument('--order', choices=['mode-first', 'path-first'], default='mode-first')
    parser.add_argument('--output', type=Path, default=Path('/tmp/simp_crab_turn_regression'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros_logs')
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from nav_msgs.msg import OccupancyGrid, Odometry
    from std_msgs.msg import Float64, String, UInt8
    from simp_planner_msgs.msg import DriveModeState, ReferencePath, ExecutionStatus, ExecutedCommand
    from rclpy.qos import QoSProfile, DurabilityPolicy
    rclpy.init()
    node = rclpy.create_node('crab_turn_regression')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    path_pub = node.create_publisher(ReferencePath, '/reference_path_data', qos)
    costmap_pub = node.create_publisher(OccupancyGrid, '/costmap', qos)
    mode_pub = node.create_publisher(UInt8, '/requested_drive_mode', qos)
    speed_pub = node.create_publisher(Float64, '/target_speed', qos)
    latest = {}
    modes = []
    def mode_callback(msg):
        latest['mode'] = msg
        if msg.status == DriveModeState.STATUS_READY and (not modes or modes[-1] != msg.current_mode):
            modes.append(msg.current_mode)
    node.create_subscription(DriveModeState, '/vehicle/drive_mode_state', mode_callback, qos)
    node.create_subscription(String, '/planner/status', lambda m: latest.update(planner=json.loads(m.data)), qos)
    node.create_subscription(ExecutionStatus, '/tracker/execution_status', lambda m: latest.update(tracker=m), 10)
    node.create_subscription(ExecutedCommand, '/planner/executed_command', lambda m: latest.update(command=m), 20)
    node.create_subscription(Odometry, '/odom', lambda m: latest.update(odom=m), 20)
    processes, logs = [], []
    report = {'pass': False, 'order': args.order}
    def wait_for(predicate, label, timeout=20.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('node exited: ' + label)
            if 'tracker' in latest and latest['tracker'].execution_state.startswith('LOCAL_STOP_'):
                raise RuntimeError(latest['tracker'].execution_state + ': ' + latest['tracker'].acceptance)
            if predicate():
                return
        raise RuntimeError('timeout: ' + label)
    def reference(mode, target):
        msg = ReferencePath(); msg.header.stamp = node.get_clock().now().to_msg(); msg.header.frame_id = 'odom'
        if mode == 2:
            msg.x = [0.] * 5; msg.y = [-3., -2., -1., 0., .2]; msg.yaw = [math.pi / 2] * 5
        else:
            msg.x = [s * math.cos(target) for s in (0., 1., 2., 3., 3.2)]
            msg.y = [s * math.sin(target) for s in (0., 1., 2., 3., 3.2)]
            msg.yaw = [target] * 5
        msg.mode = [mode] * 5
        return msg
    def ready(mode):
        m = latest.get('mode')
        return m is not None and m.current_mode == mode and m.status == m.STATUS_READY
    try:
        for package, executable, params in [
            ('planar_velocity_sim', 'planar_velocity_sim_node',
             {'initial_y': -.06, 'initial_drive_mode': 2, 'mode_transition_duration_sec': .25}),
            ('simp_tracker', 'tracker_node', {}), ('simp_planner_cpp', 'planner_node_cpp', {})]:
            command = [str(Path(get_package_prefix(package)) / 'lib' / package / executable), '--ros-args']
            for key, value in params.items():
                command += ['-p', f'{key}:={value}']
            log = (args.output / f'{executable}.log').open('w'); logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=log))
        wait_for(lambda: path_pub.get_subscription_count() > 0 and ready(2), 'discovery')
        grid = OccupancyGrid(); grid.header.frame_id = 'odom'; grid.header.stamp = node.get_clock().now().to_msg()
        grid.info.resolution = .2; grid.info.width = grid.info.height = 100
        grid.info.origin.position.x = grid.info.origin.position.y = -10.; grid.info.origin.orientation.w = 1.
        grid.data = [0] * 10000
        costmap_pub.publish(grid); mode_pub.publish(UInt8(data=2)); speed_pub.publish(Float64(data=0.))
        path_pub.publish(reference(2, 0.))
        wait_for(lambda: latest.get('planner', {}).get('terminal_hold', {}).get('latched', False), 'crab terminal hold')
        target = .7
        if args.order == 'mode-first':
            mode_pub.publish(UInt8(data=0))
            wait_for(lambda: ready(0), 'forward confirmed before new reference')
            path_pub.publish(reference(0, target))
        else:
            path_pub.publish(reference(0, target))
            # Deliberately let the spot-turn supervisor run before the external request.
            until = time.monotonic() + .15
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.01)
            mode_pub.publish(UInt8(data=0))
        wait_for(lambda: 4 in modes, 'SPOT_TURN READY')
        wait_for(lambda: modes[-1] == 0 and modes.index(4) < len(modes) - 1 and
                 latest.get('planner', {}).get('mode_control', {}).get('reference_mode') == 0,
                 'return to Forward and install new path')
        odom = latest['odom']; q = odom.pose.pose.orientation
        yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
        error = abs(math.atan2(math.sin(yaw - target), math.cos(yaw - target)))
        assert error < .04, f'returned before heading settled: {error}'
        speed_pub.publish(Float64(data=1.))
        wait_for(lambda: latest.get('command') is not None and latest['command'].vx > .1,
                 'forward motion after return')
        report.update({'pass': True, 'ready_modes': modes, 'yaw_error': error})
    except (RuntimeError, AssertionError) as error:
        report.update({'error': str(error), 'ready_modes': modes, 'planner': latest.get('planner')})
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        for log in logs:
            log.close()
        node.destroy_node(); rclpy.shutdown()
        (args.output / 'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0 if report['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
