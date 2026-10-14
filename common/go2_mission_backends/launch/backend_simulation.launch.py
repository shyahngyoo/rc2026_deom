"""Hardware-free command-driven Mission verification; no Gazebo or motor driver."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    own=get_package_share_directory('go2_mission_backends')
    mission=get_package_share_directory('robocon_mission')
    path=lambda name:os.path.join(own,'config',name)
    value=LaunchConfiguration
    simtime={'use_sim_time':ParameterValue(value('use_sim_time'),value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id',default_value='128'),
        DeclareLaunchArgument('rmw_implementation',default_value='rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('cyclonedds_uri',default_value='file://'+path('cyclonedds_sim.xml')),
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('route_file',default_value=path('backend_test_route.yaml')),
        DeclareLaunchArgument('skills_file',default_value=path('test_skills.yaml')),
        DeclareLaunchArgument('autostart',default_value='true'),
        SetEnvironmentVariable('ROS_DOMAIN_ID',value('ros_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY','1'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION',value('rmw_implementation')),
        SetEnvironmentVariable('CYCLONEDDS_URI',value('cyclonedds_uri')),
        Node(package='go2_mission_backends',executable='simulation_backend',output='screen',
             parameters=[path('simulation.yaml'),simtime,{'simulation_mode':True,
                 'publish_clock':ParameterValue(value('use_sim_time'),value_type=bool)}]),
        Node(package='go2_mission_backends',executable='robot_state_adapter',output='screen',
             parameters=[path('robot_state.yaml'),path('simulation.yaml'),simtime]),
        Node(package='go2_mission_backends',executable='localization_adapter',output='screen',
             parameters=[path('localization.yaml'),simtime]),
        Node(package='go2_mission_backends',executable='motion_arbiter',output='screen',
             parameters=[path('arbiter.yaml'),simtime]),
        Node(package='go2_mission_backends',executable='obstacle_adapter',output='screen',
             parameters=[simtime,{'simulation_mode':True,'skills_file':value('skills_file')}]),
        Node(package='go2_mission_backends',executable='preflight',output='screen',
             parameters=[path('preflight.yaml'),simtime,{'route_file':value('route_file'),
                 'require_distinct_endpoints':True}]),
        Node(package='robocon_mission',executable='mission_manager',output='screen',
             parameters=[os.path.join(mission,'config','mission_params.yaml'),
                 path('simulation.yaml'),simtime,{'route_file':value('route_file'),
                 'accepted_pose_topic':'/localization/evidence','accepted_pose_type':'evidence',
                 'accepted_pose_body_frame':'livox_frame',
                 'startup_check_service':'/mission/preflight',
                 'autostart':ParameterValue(value('autostart'),value_type=bool)}]),
    ])
