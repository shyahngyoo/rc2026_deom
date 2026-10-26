"""Real ROS timer + pseudo-terminal integration; no physical hardware."""
import os
import pty
import time
import yaml
import pytest
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('serial')
pytest.importorskip('tf2_ros')
from std_srvs.srv import Trigger
from quadruped_stm32_bridge.cybergear_node import CyberGearNode
from quadruped_stm32_bridge.cybergear_core import crc8


@pytest.mark.parametrize('enabled', [True, False])
def test_node_serial_plan_and_stale_status(tmp_path, enabled):
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    plan = tmp_path/'plan.yaml'
    plan.write_text(yaml.safe_dump({'steps':[
        {'type':'mode','key':9}, {'type':'key','key':6,'duration_s':0.15}]}))
    rclpy.init(args=['--ros-args','-p','port:='+os.ttyname(slave),
                     '-p','enable_motion:='+str(enabled).lower(),'-p','plan_file:='+str(plan)])
    node = None
    try:
        node = CyberGearNode()
        packet = bytes([165,90,97,0,0,0])
        packet += bytes([crc8(packet),13])
        received = bytearray()
        def cycle(duration, feedback=True):
            end = time.monotonic()+duration
            while time.monotonic()<end:
                if feedback:
                    os.write(master, packet)
                rclpy.spin_once(node, timeout_sec=0.02)
                try:
                    received.extend(os.read(master,4096))
                except BlockingIOError:
                    pass
        cycle(0.2)
        assert node.ready(time.monotonic())
        if not enabled:
            assert not node.start(None, Trigger.Response()).success
            node.send(6)
            cycle(0.1)
            assert bytes([83,6,69]) not in received
            return
        assert node.start(None, Trigger.Response()).success
        cycle(0.5)
        assert node.runner.state == 'completed'
        assert received.count(bytes([83,6,69])) == 1
        assert bytes([83,9,69]) in received
        assert bytes([83,5,69]) in received
        cycle(0.7, feedback=False)
        assert not node.ready(time.monotonic())
        assert not node.start(None, Trigger.Response()).success
    finally:
        if node:
            node.disconnect()
            node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)


def test_generated_course_runs_through_real_ros_and_serial(tmp_path):
    from geometry_msgs.msg import TransformStamped
    from quadruped_stm32_bridge.course import compile_course
    course={'frame':'map','obstacles':[{'id':'bridge1','name':'测试桥',
        'samples':{'origin':[0,0,0],'entry':[0,0,0],'exit':[0,0,0]},
        'steps':[{'type':'local_waypoint','target':[0,0,0],'mode_key':9},
                 {'type':'key','key':6,'duration_s':0.03}]}]}
    plan=compile_course(course)
    for step in plan['steps']:
        if 'duration_s' in step:step['duration_s']=0.03
    path=tmp_path/'generated.yaml';path.write_text(yaml.safe_dump(plan))
    master,slave=pty.openpty();os.set_blocking(master,False)
    rclpy.init(args=['--ros-args','-p','port:='+os.ttyname(slave),
                    '-p','enable_motion:=true','-p','plan_file:='+str(path)])
    node=None
    try:
        node=CyberGearNode()
        body=bytes([165,90,97,0,0,0]);packet=body+bytes([crc8(body),13])
        received=bytearray()
        def cycle(duration):
            end=time.monotonic()+duration
            while time.monotonic()<end:
                t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_footprint'
                t.header.stamp=node.get_clock().now().to_msg();t.transform.rotation.w=1.
                node.tf.set_transform(t,'test')
                os.write(master,packet);rclpy.spin_once(node,timeout_sec=0.02)
                try:received.extend(os.read(master,4096))
                except BlockingIOError:pass
        cycle(0.2)
        assert node.start(None,Trigger.Response()).success
        cycle(1.2)
        assert node.runner.state=='completed'
        assert received.count(bytes([83,6,69]))==1
        assert all(s['obstacle']=='测试桥' for s in node.runner.steps)
    finally:
        if node:node.disconnect();node.destroy_node()
        rclpy.shutdown();os.close(master);os.close(slave)
