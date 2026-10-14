#!/usr/bin/env python3
"""Minimal Mission Manager handoff: transit, approach, then external skill."""

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from go2_obstacle_interfaces.action import ExecuteObstacle
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Trigger


def pose_goal(node, values, tree):
    x, y, yaw = values
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
    goal.behavior_tree = str(tree)
    return goal


def await_future(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.done()


def run_action(node, client, goal, timeout, label):
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError(f"{label}: Action server unavailable")
    sent = client.send_goal_async(goal)
    if not await_future(node, sent, 10.0):
        raise RuntimeError(f"{label}: goal acceptance timed out")
    handle = sent.result()
    if not handle.accepted:
        raise RuntimeError(f"{label}: goal rejected")
    result_future = handle.get_result_async()
    if not await_future(node, result_future, timeout):
        cancel_future = handle.cancel_goal_async()
        await_future(node, cancel_future, 5.0)
        await_future(node, result_future, 5.0)
        raise RuntimeError(f"{label}: timed out; cancellation requested")
    response = result_future.result()
    if response.status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError(f"{label}: terminal status {response.status}")
    return response.result


def wait_until_stable(node, last_odom, timeout=8.0):
    """Require fresh odometry below both speed limits for a full second."""
    deadline = time.monotonic() + timeout
    stable_since = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        sample, stamp = last_odom
        fresh = sample is not None and time.monotonic() - stamp <= 0.5
        if fresh:
            twist = sample.twist.twist
            slow = (math.hypot(twist.linear.x, twist.linear.y) <= 0.03 and
                    abs(twist.angular.z) <= 0.05)
        else:
            slow = False
        if slow:
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= 1.0:
                return True
        else:
            stable_since = None
    return False


def localization_ok(node, service_name):
    """External localization supervisor owns this check, not the skill."""
    client = node.create_client(Trigger, service_name)
    if not client.wait_for_service(timeout_sec=5.0):
        return False
    future = client.call_async(Trigger.Request())
    return await_future(node, future, 5.0) and future.result().success


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transit", nargs=3, type=float, required=True,
                        metavar=("X", "Y", "YAW"))
    parser.add_argument("--approach", nargs=3, type=float, required=True,
                        metavar=("X", "Y", "YAW"))
    parser.add_argument("--obstacle-id", required=True)
    parser.add_argument("--skill-id", required=True)
    parser.add_argument("--next-transit", nargs=3, type=float,
                        metavar=("X", "Y", "YAW"))
    parser.add_argument("--localization-ok-service",
                        default="/mission/localization_ok")
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    values = [*args.transit, *args.approach, *(args.next_transit or [])]
    if not all(math.isfinite(value) for value in values):
        parser.error("all coordinates must be finite")
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be positive")

    share = Path(get_package_share_directory("go2_nav_bringup"))
    transit_tree = share / "behavior_trees" / "nav_transit.xml"
    approach_tree = share / "behavior_trees" / "nav_approach.xml"
    if not transit_tree.is_file() or not approach_tree.is_file():
        parser.error("installed behavior trees are missing")

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("mission_handoff_example")
    last_odom = [None, 0.0]

    def on_odom(message):
        last_odom[0] = message
        last_odom[1] = time.monotonic()

    subscription = node.create_subscription(Odometry, "/odom", on_odom, 10)
    nav = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    skill = ActionClient(node, ExecuteObstacle, "/execute_obstacle")
    try:
        run_action(node, nav, pose_goal(node, args.transit, transit_tree),
                   args.timeout, "transit")
        run_action(node, nav, pose_goal(node, args.approach, approach_tree),
                   args.timeout, "approach")
        if not wait_until_stable(node, last_odom):
            raise RuntimeError("robot did not settle; obstacle skill withheld")
        goal = ExecuteObstacle.Goal(
            obstacle_id=args.obstacle_id, skill_id=args.skill_id
        )
        result = run_action(node, skill, goal, args.timeout, "obstacle skill")
        if not result.completed or not result.control_released:
            raise RuntimeError("skill did not complete and release control")
        print("SKILL_COMPLETE_CONTROL_RELEASED", flush=True)
        if args.next_transit:
            if not localization_ok(node, args.localization_ok_service):
                raise RuntimeError("localization not confirmed; navigation withheld")
            run_action(node, nav,
                       pose_goal(node, args.next_transit, transit_tree),
                       args.timeout, "next transit")
        else:
            print("NEXT_NAV_REQUIRES_LOCALIZATION_CHECK", flush=True)
        return 0
    except RuntimeError as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
