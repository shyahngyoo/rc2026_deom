"""Actual ROS safety-diagnostic checks; no sensor driver or motor is started."""
import os
import sys
import time
from pathlib import Path
os.environ.setdefault('ROS_DOMAIN_ID','168')
os.environ['ROS_LOCALHOST_ONLY']='1'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from go2_obstacle_interfaces.msg import MotionTelemetry
from go2_onboard_bringup.profile import create_profile
from go2_onboard_bringup.doctor_node import OnboardDoctor


def test_live_mock_feedback_cannot_unlock_real_motion(tmp_path):
    folder=create_profile(tmp_path/'profile','192.168.1.50','192.168.1.111',[.11,-.07,.08,.01,.02,-.03])
    context=Context();rclpy.init(context=context)
    doctor=OnboardDoctor(context=context,parameter_overrides=[Parameter('config_dir',value=str(folder)),Parameter('mode',value='lidar')])
    sender=Node('doctor_safety_probe',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(doctor);executor.add_node(sender)
    pub=sender.create_publisher(MotionTelemetry,'/motion/telemetry',10)
    client=sender.create_client(Trigger,'/onboard/doctor')
    def spin(condition,timeout=3.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
            if condition():return
        assert condition()
    try:
        spin(lambda:pub.get_subscription_count()==1 and client.service_is_ready())
        m=MotionTelemetry();m.header.stamp=sender.get_clock().now().to_msg();m.header.frame_id='base_footprint'
        m.source_id='simulated_feedback_cannot_authorize';m.sequence=1;m.known=True;m.safe_stance=True;m.control_releasable=True;m.stopped=True
        pub.publish(m);spin(lambda:doctor.telemetry is not None)
        result=doctor.report()
        assert result['live_checks']['measured_low_level_state']
        assert result['status']=='NOT_READY' and not result['mission_allowed'] and not result['motion_enabled']
        response=client.call_async(Trigger.Request());spin(response.done)
        assert response.result().success is False
        assert sender.count_publishers('/cmd_vel')==0
        assert sender.count_publishers('/onboard/nav_cmd_raw')==0
        assert not doctor.get_publishers_info_by_topic('/motion/authority')
        # Old or incorrectly framed source cannot be counted as fresh measured state.
        m.sequence=2;m.header.frame_id='odom';pub.publish(m)
        spin(lambda:doctor.telemetry is None)
        assert not doctor.report()['live_checks']['measured_low_level_state']
    finally:
        executor.shutdown();doctor.destroy_node();sender.destroy_node();context.shutdown()
