from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    map_file = LaunchConfiguration("map_file")
    target_speed = LaunchConfiguration("target_speed")
    save_period = LaunchConfiguration("save_period")
    debug_output_dir = LaunchConfiguration("debug_output_dir")
    kinematics_model = LaunchConfiguration("kinematics_model")

    default_map = PathJoinSubstitution(
        [FindPackageShare("simp_planner_tools"), "maps", "stadium_track.csv"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_file", default_value=default_map),
            DeclareLaunchArgument("target_speed", default_value="2.0"),
            DeclareLaunchArgument("save_period", default_value="10.0"),
            DeclareLaunchArgument("tracking_enabled", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("tracking_longitudinal_kp", default_value="0.8"),
            DeclareLaunchArgument("tracking_lateral_kp", default_value="0.6"),
            DeclareLaunchArgument("tracking_heading_kp", default_value="1.5"),
            DeclareLaunchArgument(
                "kinematics_model",
                default_value="ideal",
                choices=["ideal", "noisy"],
            ),
            DeclareLaunchArgument(
                "debug_output_dir",
                default_value="/home/sum/Desktop/simp_planner/simp_planner_debug",
            ),
            Node(
                package="planar_velocity_sim",
                executable="planar_velocity_sim_node",
                name="planar_velocity_sim_node",
                output="screen",
                parameters=[{"kinematics_model": kinematics_model}],
            ),
            Node(
                package="simp_planner_tools",
                executable="track_map_provider_node",
                name="track_map_provider_node",
                output="screen",
                parameters=[
                    {
                        "map_file": map_file,
                        "target_speed": ParameterValue(target_speed, value_type=float),
                    }
                ],
            ),
            Node(
                package="simp_planner_cpp",
                executable="planner_node_cpp",
                name="planner_node_cpp",
                output="screen",
                parameters=[{
                    "tracking_enabled": ParameterValue(LaunchConfiguration("tracking_enabled"), value_type=bool),
                    **{
                        name: ParameterValue(LaunchConfiguration(name), value_type=float)
                        for name in ("tracking_longitudinal_kp", "tracking_lateral_kp", "tracking_heading_kp")
                    },
                }],
            ),
            Node(
                package="simp_planner_tools",
                executable="debug_plot_node",
                name="debug_plot_node",
                output="screen",
                parameters=[
                    {
                        "scenario": "track_map",
                        "save_period": ParameterValue(save_period, value_type=float),
                        "output_dir": debug_output_dir,
                    }
                ],
            ),
        ]
    )
