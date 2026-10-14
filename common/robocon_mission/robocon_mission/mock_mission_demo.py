"""Isolated in-process ROS demo: simulated Actions/TF only, no actuator outputs."""
import argparse
import math
import time
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from go2_obstacle_interfaces.action import ExecuteObstacle
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from .mission_fsm import State
from .mission_manager import MissionManager
from .mock_motion_services import MockMotionServices


class MockEnvironment(Node):
    def __init__(self, move_skill=False):
        super().__init__("mock_mission_environment")
        self.move_skill = move_skill
        self.pose = (0.0, 0.0, 0.0)
        self.control = MockMotionServices(self)
        self.group = ReentrantCallbackGroup()
        self.nav = ActionServer(self, NavigateToPose, "/navigate_to_pose", self._nav,
                                callback_group=self.group)
        self.skill = ActionServer(
            self, ExecuteObstacle, "/execute_obstacle", self._skill,
            goal_callback=lambda _: GoalResponse.ACCEPT if self.control.owner == "SKILL"
            else GoalResponse.REJECT,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)
        self.health = self.create_service(Trigger, "/mission/localization_ok", self._health)
        self.odom = self.create_publisher(Odometry, "/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self._sensors, clock=self.clock)

    def _health(self, _req, res):
        res.success, res.message = True, "SIMULATION ONLY localization health"
        return res

    def _nav(self, handle):
        if self.control.owner != "NAVIGATION":
            handle.abort()
        else:
            pose = handle.request.pose.pose
            q = pose.orientation
            self.pose = (pose.position.x, pose.position.y,
                         math.atan2(2*q.w*q.z, 1 - 2*q.z*q.z))
            handle.succeed()
        return NavigateToPose.Result()

    async def _skill(self, handle):
        self.control.skill_active = True
        future = Future()
        deadline = time.monotonic() + 0.3

        def finish():
            if not handle.is_cancel_requested and time.monotonic() < deadline:
                return
            canceled = handle.is_cancel_requested
            if canceled:
                handle.canceled()
            else:
                # Independent simulated physical displacement, never read end_pose.
                if self.move_skill:
                    self.pose = (self.pose[0] + 0.4, self.pose[1], self.pose[2])
                handle.succeed()
            self.control.skill_active = False
            timer.cancel()
            self.destroy_timer(timer)
            future.set_result(ExecuteObstacle.Result(
                completed=not canceled, safe_stance=True, control_released=True,
                message="SIMULATION ONLY; displacement enabled" if self.move_skill
                else "SIMULATION ONLY; skill does not move robot"))
        timer = self.create_timer(0.02, finish, clock=self.clock)
        return await future

    def _sensors(self):
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id = stamp, "map"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x, tf.transform.translation.y = self.pose[:2]
        tf.transform.rotation.z = math.sin(self.pose[2]/2)
        tf.transform.rotation.w = math.cos(self.pose[2]/2)
        self.tf.sendTransform(tf)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        self.odom.publish(odom)

    def destroy_servers(self):
        self.nav.destroy()
        self.skill.destroy()
        self.control.destroy()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--move-skill", action="store_true")
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    rclpy.init(args=args)
    environment = MockEnvironment(options.move_skill)
    route = Path(get_package_share_directory("robocon_mission")) / "config" / "mock_closed_loop_route.yaml"
    mission = MissionManager(parameter_overrides=[
        Parameter(name, value=value) for name, value in {
            "route_file": str(route), "simulation_mode": True, "use_sim_time": False,
            "autostart": True, "stable_duration": 0.2, "verify_stable_duration": 0.2,
            "server_timeout": 2.0, "cancel_timeout": 1.0, "handoff_timeout": 2.0,
        }.items()])
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(environment)
    executor.add_node(mission)
    try:
        while rclpy.ok() and mission.fsm.state not in (State.FINISHED, State.ABORTED):
            executor.spin_once(timeout_sec=0.1)
        # Keep the independent mock arbiter spinning through final hold acknowledgment.
        deadline = time.monotonic() + 1.5
        while rclpy.ok() and time.monotonic() < deadline and mission.handoff.emergency.pending:
            executor.spin_once(timeout_sec=0.05)
        print(f"MOCK RESULT: {mission.fsm.state.value}; completed={mission.route.completed}; "
              f"reason={mission.fsm.error}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(mission)
        mission.close_safely()
        executor.shutdown(timeout_sec=2.0)
        environment.destroy_servers()
        mission.destroy_node()
        environment.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

