"""Real sensor/localization composition and strictly isolated planning diagnostics."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import IncludeLaunchDescription, GroupAction, LogInfo, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from .profile import load_profile


def include(package,filename,arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package),'launch',filename)),
        launch_arguments={key:str(value) for key,value in arguments.items()}.items())


def planning_actions(config_dir):
    """Public test factory: never starts LiDAR, localization or motor backends.

    Controller/behavior internal cmd input is distinct from smoother output.
    Global remaps cover relative and absolute names inherited by the full Nav2
    include, including behavior_server and velocity_smoother. No motor remap
    switch is provided. Real-world collision safety is not certified here.
    """
    profile,folder=load_profile(config_dir,'navigation')
    nav_share=get_package_share_directory('nav2_bringup')
    from nav2_common.launch import RewrittenYaml
    nav_params=RewrittenYaml(source_file=str(folder/'nav2.yaml'),param_rewrites={
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml':os.path.join(
            get_package_share_directory('go2_nav_bringup'),'behavior_trees','nav_transit.xml')},convert_types=True)
    commands=[SetRemap(src,dst) for src,dst in [
        ('cmd_vel','/onboard/nav_cmd_internal'),('/cmd_vel','/onboard/nav_cmd_internal'),
        ('cmd_vel_nav','/onboard/nav_cmd_internal'),('/cmd_vel_nav','/onboard/nav_cmd_internal'),
        ('cmd_vel_smoothed','/onboard/nav_cmd_raw'),('/cmd_vel_smoothed','/onboard/nav_cmd_raw')]]
    nodes=[Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',
        parameters=[str(folder/'nav2.yaml'),dict(use_sim_time=False,yaml_filename=profile['nav_map'])]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='onboard_map_lifecycle',output='screen',
             parameters=[dict(use_sim_time=False,autostart=True,node_names=['map_server'])]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav_share,'launch','navigation_launch.py')),
             launch_arguments=dict(use_sim_time='false',autostart='true',use_composition='False',
                                   params_file=nav_params).items())]
    return [AppendEnvironmentVariable('LD_LIBRARY_PATH',os.path.join(get_package_prefix('go2_nav_bringup'),'lib'),prepend=True),
            LogInfo(msg='PLANNING DIAGNOSTICS ONLY: Nav2 velocity topics isolated; robot /cmd_vel has no output path.'),
            GroupAction(actions=commands+nodes)]


def onboard_actions(config_dir,mode):
    profile,folder=load_profile(config_dir,mode)
    actions=[LogInfo(msg='REAL INPUTS; MOTION DISABLED; no Mission/mock/simulated low-level nodes.'),
        Node(package='go2_onboard_bringup',executable='doctor_node',name='onboard_doctor',output='screen',
             parameters=[dict(config_dir=str(folder),mode=mode,use_sim_time=False)])]
    global_mode=mode in ('localization','navigation')
    odometry_only=profile.get('localization_source')=='odometry'
    if global_mode and not odometry_only:
        actions.append(include('go2_real_bringup','localization.launch.py',dict(map_path=profile['map_pcd'],
            show_rviz='false',params_file=str(folder/'gicp.yaml'),lio_params=str(folder/'fast_lio.yaml'),
            user_config_path=str(folder/'mid360.json'))))
    else:
        actions.append(include('go2_real_bringup','lidar.launch.py',dict(user_config_path=str(folder/'mid360.json'),xfer_format='1')))
        if mode=='odometry' or global_mode:actions.append(include('go2_real_bringup','odometry.launch.py',dict(show_rviz='false',lio_params=str(folder/'fast_lio.yaml'))))
    if global_mode and odometry_only:
        actions.append(Node(package='go2_mission_backends',executable='odometry_localization',
            name='odometry_localization',output='screen',parameters=[dict(
                odom_in_map=[float(v) for v in profile['odom_in_map']],alignment_confirmed=True,use_sim_time=False)]))
    if mode in ('odometry','localization','navigation'):
        actions.append(Node(package='go2_mission_backends',executable='robot_state_adapter',name='robot_state_adapter',
            output='screen',parameters=[str(folder/'robot_state.yaml'),dict(use_sim_time=False)]))
    if mode in ('localization','navigation'):
        actions.extend([
            Node(package='go2_mission_backends',executable='localization_adapter',name='localization_evidence_adapter',
                 output='screen',parameters=[str(folder/'localization.yaml'),dict(use_sim_time=False)]),
            Node(package='go2_mission_backends',executable='preflight',name='mission_preflight',output='screen',
                 parameters=[dict(use_sim_time=False,route_file=str(folder/'preflight_route.yaml'),
                     output_topic='/cmd_vel',expected_output_node='__MOTOR_OUTPUT_DISABLED__',
                     required_nav_nodes=['/controller_server','/planner_server','/bt_navigator'])])])
    if mode=='navigation':
        actions.append(Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',
            name='onboard_cloud_to_scan',output='screen',parameters=[str(folder/'scan.yaml'),dict(use_sim_time=False)],
            remappings=[('cloud_in','/cloud_registered_body'),('scan','/onboard/scan_diagnostic')]))
        actions.extend(planning_actions(folder))
    return actions
