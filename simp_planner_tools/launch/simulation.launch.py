import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from simp_planner_tools.scenario_definition import load_scenario_definition


def _resolved_nodes(context):
    scenario_name = LaunchConfiguration("scenario").perform(context)
    target_speed = float(LaunchConfiguration("target_speed").perform(context))
    save_period = float(LaunchConfiguration("save_period").perform(context))
    debug_output_dir = LaunchConfiguration("debug_output_dir").perform(context)
    path_update_distance = float(
        LaunchConfiguration("path_update_distance").perform(context)
    )
    mode_transition_duration = float(
        LaunchConfiguration("mode_transition_duration_sec").perform(context)
    )
    command_frequency_hz = float(
        LaunchConfiguration("command_frequency_hz").perform(context)
    )
    requested_footprint_circle_count = int(
        LaunchConfiguration("oriented_footprint_circle_count").perform(context)
    )
    requested_costmap_resolution = float(
        LaunchConfiguration("costmap_resolution").perform(context)
    )
    footprint_translation_step = float(
        LaunchConfiguration("oriented_footprint_translation_step_m").perform(context)
    )
    footprint_yaw_step = float(
        LaunchConfiguration("oriented_footprint_yaw_step_deg").perform(context)
    )

    share_directory = get_package_share_directory("simp_planner_tools")
    scenario = load_scenario_definition(share_directory, scenario_name)
    footprint_circle_count = (
        int(scenario.footprint_circle_count)
        if requested_footprint_circle_count <= 0
        else requested_footprint_circle_count
    )
    costmap_resolution = (
        float(scenario.costmap_resolution)
        if requested_costmap_resolution <= 0.0
        else requested_costmap_resolution
    )
    first_path = scenario.phases[0].path
    first_mode = int(first_path.mode[0])
    mode_heading_offset = {
        0: 0.0,
        1: math.pi,
        2: 0.5 * math.pi,
        3: -0.5 * math.pi,
    }[first_mode]
    initial_body_yaw = float(first_path.yaw[0]) - mode_heading_offset

    return [
        Node(
            package="planar_velocity_sim",
            executable="planar_velocity_sim_node",
            name="planar_velocity_sim_node",
            output="screen",
            parameters=[
                {
                    "initial_x": float(first_path.x[0]),
                    "initial_y": float(first_path.y[0]),
                    "initial_yaw": initial_body_yaw,
                    "initial_drive_mode": first_mode,
                    "mode_transition_duration_sec": mode_transition_duration,
                }
            ],
        ),
        Node(
            package="simp_planner_tools",
            executable="scenario_manager_node",
            name="scenario_manager_node",
            output="screen",
            parameters=[
                {
                    "scenario": scenario_name,
                    "target_speed": target_speed,
                    "path_update_distance": path_update_distance,
                    "costmap_resolution": costmap_resolution,
                }
            ],
        ),
        Node(
            package="simp_planner_cpp",
            executable="planner_node_cpp",
            name="planner_node_cpp",
            output="screen",
            parameters=[
                {
                    "command_frequency_hz": command_frequency_hz,
                    "oriented_footprint_circle_count": footprint_circle_count,
                    "oriented_footprint_translation_step_m": footprint_translation_step,
                    "oriented_footprint_yaw_step_deg": footprint_yaw_step,
                }
            ],
        ),
        Node(
            package="simp_planner_tools",
            executable="debug_plot_node",
            name="debug_plot_node",
            output="screen",
            parameters=[
                {
                    "scenario": scenario_name,
                    "save_period": save_period,
                    "output_dir": debug_output_dir,
                }
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="stadium"),
            DeclareLaunchArgument(
                "target_speed",
                default_value="-1.0",
                description="Negative value uses the scenario default speed.",
            ),
            DeclareLaunchArgument("path_update_distance", default_value="1.0"),
            DeclareLaunchArgument(
                "mode_transition_duration_sec",
                default_value="2.0",
                description="Vehicle-side drive-mode transition duration.",
            ),
            DeclareLaunchArgument("command_frequency_hz", default_value="100.0"),
            DeclareLaunchArgument(
                "oriented_footprint_circle_count", default_value="0",
                description="0 uses the scenario-recommended footprint resolution.",
            ),
            DeclareLaunchArgument(
                "costmap_resolution", default_value="-1.0",
                description="Negative value uses the scenario-recommended costmap resolution.",
            ),
            DeclareLaunchArgument(
                "oriented_footprint_translation_step_m", default_value="0.20"
            ),
            DeclareLaunchArgument("oriented_footprint_yaw_step_deg", default_value="2.0"),
            DeclareLaunchArgument("save_period", default_value="10.0"),
            DeclareLaunchArgument(
                "debug_output_dir",
                default_value="/home/sum/Desktop/simp_planner/simp_planner_debug",
            ),
            OpaqueFunction(function=_resolved_nodes),
        ]
    )
