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

    default_map = PathJoinSubstitution(
        [FindPackageShare("simp_planner_tools"), "maps", "stadium_track.csv"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_file", default_value=default_map),
            DeclareLaunchArgument("target_speed", default_value="2.0"),
            DeclareLaunchArgument("save_period", default_value="10.0"),
            DeclareLaunchArgument(
                "debug_output_dir",
                default_value="/home/sum/Desktop/simp_planner/simp_planner_debug",
            ),
            Node(
                package="planar_velocity_sim",
                executable="planar_velocity_sim_node",
                name="planar_velocity_sim_node",
                output="screen",
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
