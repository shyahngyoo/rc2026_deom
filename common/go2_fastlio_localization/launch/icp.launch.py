"""ICP and planar odometry bridge; requires an existing FAST-LIO."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    map_file = LaunchConfiguration("map_file")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    auto_initial_delay = LaunchConfiguration("auto_initial_delay")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    fast_lio_odom_bridge = Node(
        package="go2_fastlio_localization",
        executable="fast_lio_odom_bridge",
        name="fast_lio_odom_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    localizer = Node(
        package="go2_fastlio_localization",
        executable="pcd_icp_localizer",
        name="pcd_icp_localizer",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_file": map_file,
                "auto_initialize": ParameterValue(
                    auto_initial_pose, value_type=bool
                ),
                "auto_initialize_delay": ParameterValue(
                    auto_initial_delay, value_type=float
                ),
                "initial_x": ParameterValue(initial_x, value_type=float),
                "initial_y": ParameterValue(initial_y, value_type=float),
                "initial_yaw": ParameterValue(initial_yaw, value_type=float),
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_file", description="Required reference PCD map"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("auto_initial_pose", default_value="false"),
        DeclareLaunchArgument("auto_initial_delay", default_value="10.0"),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        fast_lio_odom_bridge, localizer,
    ])
