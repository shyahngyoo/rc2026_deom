#!/usr/bin/env python3
"""Simulation-only obstacle skill Action server; it never commands the robot."""

import argparse
import sys
import time

import rclpy
from go2_obstacle_interfaces.action import ExecuteObstacle
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


class MockObstacleSkill(Node):
    def __init__(self, duration, fail, hold_control):
        super().__init__("mock_obstacle_skill")
        self.duration = duration
        self.fail = fail
        self.hold_control = hold_control
        self.server = ActionServer(
            self, ExecuteObstacle, "/execute_obstacle", self.execute,
            goal_callback=self.accept_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

    def accept_goal(self, request):
        if not request.obstacle_id or not request.skill_id:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, handle):
        self.get_logger().info(
            f"Mock only: {handle.request.skill_id} at {handle.request.obstacle_id}"
        )
        started = time.monotonic()
        deadline = started + self.duration
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                handle.canceled()
                return ExecuteObstacle.Result(
                    completed=False, control_released=True, message="canceled"
                )
            progress = min(1.0, (time.monotonic() - started) / self.duration)
            phase = ("preparing" if progress < 0.2 else
                     "executing" if progress < 0.8 else "settling")
            handle.publish_feedback(ExecuteObstacle.Feedback(
                phase=phase, progress=progress))
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

        if self.fail:
            handle.abort()
            return ExecuteObstacle.Result(
                completed=False, control_released=True, message="simulated failure"
            )
        handle.succeed()
        self.get_logger().info(
            f"Mock complete: {handle.request.obstacle_id}; no robot displacement")
        return ExecuteObstacle.Result(
            completed=True, control_released=not self.hold_control, safe_stance=True,
            message="SIMULATION ONLY; no displacement" if not self.hold_control else "control held",
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--fail", action="store_true")
    parser.add_argument("--hold-control", action="store_true")
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if not 0.0 <= args.duration <= 60.0:
        parser.error("--duration must be in [0, 60] seconds")
    rclpy.init(args=sys.argv)
    node = MockObstacleSkill(args.duration, args.fail, args.hold_control)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
