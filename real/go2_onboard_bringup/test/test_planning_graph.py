"""Real Nav2 processes + dummy TF/grid: final output must stay private.

Run after building this package and sourcing its overlay. No MID360 driver,
GICP, robot simulator or motor backend is launched. Static TF and independently
published zero odometry are test inputs; a goal is canceled after actual private
command output is observed. This validates remaps at the ROS graph, not by AST.
"""
import os
import json
import signal
import subprocess
import sys
import time
from pathlib import Path
import pytest
import yaml
os.environ.setdefault('ROS_DOMAIN_ID','169')
os.environ['ROS_LOCALHOST_ONLY']='1'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from go2_onboard_bringup.profile import create_profile

LAUNCHER = """
import sys
from launch import LaunchDescription,LaunchService
from launch_ros.actions import Node
from go2_onboard_bringup.launching import planning_actions
nodes=planning_actions(sys.argv[1])
for parent,child in [('map','nav_odom'),('nav_odom','base_footprint')]:
 nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',
  name='planning_test_'+child,arguments=['--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0',
  '--frame-id',parent,'--child-frame-id',child],parameters=[{'use_sim_time':False}]))
service=LaunchService();service.include_launch_description(LaunchDescription(nodes))
sys.exit(service.run())
"""


def test_actual_nav2_graph_has_no_motor_publisher(tmp_path):
    pcd=tmp_path/'test.pcd';pcd.write_text('FIELDS x y z\nPOINTS 1\nDATA ascii\n0 0 0\n')
    image=tmp_path/'map.pgm';image.write_bytes(b'P5\n100 100\n255\n'+bytes([254])*10000)
    grid=tmp_path/'map.yaml';grid.write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.1,origin=[-5.,-5.,0.],negate=0,occupied_thresh=.65,free_thresh=.25)))
    folder=create_profile(tmp_path/'profile','192.168.1.50','192.168.1.111',[.11,-.07,.08,.01,.02,-.03],
        stage='localization',map_pcd=str(pcd),initial_pose=[0.]*6,nav_map=str(grid),
        measured_footprint=[[-.4,-.2],[.4,-.2],[.4,.2],[-.4,.2]])
    context=Context();rclpy.init(context=context)
    probe=Node('planning_isolation_probe',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(probe)
    odom_pub=probe.create_publisher(Odometry,'/odom',10)
    def odom():
        m=Odometry();m.header.stamp=probe.get_clock().now().to_msg();m.header.frame_id='nav_odom';m.child_frame_id='base_footprint';m.pose.pose.orientation.w=1.;odom_pub.publish(m)
    timer=probe.create_timer(.03,odom)
    raw=[];internal=[]
    probe.create_subscription(Twist,'/onboard/nav_cmd_raw',raw.append,50)
    probe.create_subscription(Twist,'/onboard/nav_cmd_internal',internal.append,50)
    client=ActionClient(probe,NavigateToPose,'/navigate_to_pose')
    log_path=tmp_path/'nav2_graph.log';log=log_path.open('w')
    process=subprocess.Popen(['/usr/bin/python3','-u','-c',LAUNCHER,str(folder)],env=dict(os.environ),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    handle=None
    def wait(condition,timeout=30.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
            assert process.poll() is None,log_path.read_text()
            assert 'process has died' not in log_path.read_text(),log_path.read_text()
            assert probe.count_publishers('/cmd_vel')==0,'Nav2 published to robot /cmd_vel'
            if condition():return
        assert condition(),log_path.read_text()
    try:
        wait(lambda:client.server_is_ready() and probe.count_publishers('/onboard/nav_cmd_raw')>=1)
        lifecycle={name:probe.create_client(GetState,'/'+name+'/get_state') for name in
            ['map_server','controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother']}
        states={};pending={}
        def active():
            for name,service in lifecycle.items():
                future=pending.get(name)
                if future is not None and future.done():
                    states[name]=future.result().current_state.id;pending.pop(name)
                if name not in pending and service.service_is_ready():pending[name]=service.call_async(GetState.Request())
            return all(states.get(name)==3 for name in lifecycle)
        wait(active,timeout=30.)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=probe.get_clock().now().to_msg()
        goal.pose.pose.position.x=.6;goal.pose.pose.orientation.w=1.
        future=client.send_goal_async(goal);wait(future.done)
        handle=future.result();assert handle.accepted
        wait(lambda:any(abs(m.linear.x)+abs(m.linear.y)+abs(m.angular.z)>1e-6 for m in raw),timeout=15.)
        assert internal and probe.count_publishers('/cmd_vel')==0
        publishers=probe.get_publishers_info_by_topic('/onboard/nav_cmd_raw')
        assert any(p.node_name=='velocity_smoother' for p in publishers),publishers
        assert not probe.get_publishers_info_by_topic('/motion/authority')
        cancel=handle.cancel_goal_async();wait(cancel.done,timeout=5.)
        assert cancel.result().goals_canceling
        (tmp_path/'graph_report.json').write_text(json.dumps(dict(success=True,lifecycle_states=states,
            cmd_vel_publishers=probe.count_publishers('/cmd_vel'),goal_accepted=handle.accepted,cancel_confirmed=True,
            raw_publishers=[p.node_name for p in publishers],raw_command_samples=len(raw),internal_command_samples=len(internal),
            actual_nonzero_raw_observed=True,ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
            rmw_implementation=os.environ.get('RMW_IMPLEMENTATION'),cyclonedds_uri=os.environ.get('CYCLONEDDS_URI'),
            ament_prefix_path=os.environ.get('AMENT_PREFIX_PATH'),sensors_started=False,motors_started=False),indent=2))
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3.)
        log.close();executor.shutdown();probe.destroy_node();context.shutdown()
