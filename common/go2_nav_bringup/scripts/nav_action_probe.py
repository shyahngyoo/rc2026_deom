#!/usr/bin/env python3
"""Small upper-layer probe for the transit/approach NavigateToPose contract."""

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformListener


STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
}


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("expected a finite positive number")
    return number


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send transit then approach goals using the installed Nav2 BTs."
    )
    parser.add_argument(
        "--goal", action="append", nargs=4, required=True,
        metavar=("MODE", "X", "Y", "YAW"),
        help="Repeat for each goal; MODE is transit or approach, YAW is radians.",
    )
    parser.add_argument("--frame", default="map")
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--lifecycle-service", default="/bt_navigator/get_state")
    parser.add_argument("--skip-lifecycle-check", action="store_true")
    parser.add_argument("--server-timeout", type=positive_float, default=30.0)
    parser.add_argument("--goal-timeout", type=positive_float, default=120.0)
    parser.add_argument("--cancel-grace", type=positive_float, default=10.0)
    parser.add_argument("--check-accuracy", action="store_true")
    parser.add_argument("--settle-seconds", type=positive_float, default=1.0)
    parser.add_argument(
        "--cancel-after", type=positive_float,
        help="Request cancellation this many seconds after each accepted goal.",
    )
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if not args.frame:
        parser.error("--frame must not be empty")
    goals = []
    for mode, x_text, y_text, yaw_text in args.goal:
        if mode not in ("transit", "approach"):
            parser.error("MODE must be transit or approach")
        try:
            x, y, yaw = map(float, (x_text, y_text, yaw_text))
        except ValueError:
            parser.error("X, Y and YAW must be numbers")
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            parser.error("X, Y and YAW must be finite")
        goals.append((mode, x, y, yaw))
    return args, goals


def spin_until(node, future, deadline):
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(
            node, timeout_sec=max(0.0, min(0.1, deadline - time.monotonic()))
        )
    return future.done()


def wait_for_nav_active(node, service_name, timeout):
    client = node.create_client(GetState, service_name)
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if not client.wait_for_service(timeout_sec=1.0):
            continue
        state_future = client.call_async(GetState.Request())
        if spin_until(node, state_future, min(deadline, time.monotonic() + 2.0)):
            try:
                if state_future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                    return True
            except Exception:
                pass
        rclpy.spin_once(node, timeout_sec=0.2)
    return False


def cancel_and_wait(node, goal_handle, result_future, grace):
    deadline = time.monotonic() + grace
    cancel_future = goal_handle.cancel_goal_async()
    if not spin_until(node, cancel_future, deadline):
        print("CANCEL_RESPONSE_TIMEOUT", flush=True)
        return False
    response = cancel_future.result()
    if not response.goals_canceling:
        print("CANCEL_NOT_ACCEPTED", flush=True)
    if not spin_until(node, result_future, deadline):
        print("CANCEL_TERMINAL_TIMEOUT", flush=True)
        return False
    return True


def verify_accuracy(node, tf_buffer, last_odom, frame, goal_data, options):
    mode, x, y, yaw = goal_data
    deadline = time.monotonic() + options.settle_seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(
            node, timeout_sec=max(0.0, min(0.1, deadline - time.monotonic()))
        )
    try:
        transform = tf_buffer.lookup_transform(frame, "base_footprint", Time())
    except Exception as error:
        print(f"ACCURACY_TF_UNAVAILABLE mode={mode} error={error}", flush=True)
        return False
    pose = transform.transform
    measured_yaw = math.atan2(
        2.0 * (pose.rotation.w * pose.rotation.z +
               pose.rotation.x * pose.rotation.y),
        1.0 - 2.0 * (pose.rotation.y ** 2 + pose.rotation.z ** 2),
    )
    xy_error = math.hypot(pose.translation.x - x, pose.translation.y - y)
    yaw_error = abs(math.atan2(math.sin(measured_yaw - yaw),
                               math.cos(measured_yaw - yaw)))
    if last_odom[0] is None or time.monotonic() - last_odom[1] > 1.0:
        print(f"ACCURACY_ODOM_UNAVAILABLE mode={mode}", flush=True)
        return False
    twist = last_odom[0].twist.twist
    linear_speed = math.hypot(twist.linear.x, twist.linear.y)
    angular_speed = abs(twist.angular.z)
    # Check settled entry accuracy against Mission acceptance, not Nav2's tighter yaw goal.
    xy_limit, yaw_limit = (0.10, 0.10) if mode == "approach" else (0.25, 0.26)
    within_limit = (xy_error <= xy_limit and yaw_error <= yaw_limit and
                    linear_speed <= 0.03 and angular_speed <= 0.05)
    print(
        f"ACCURACY mode={mode} xy_error={xy_error:.4f} "
        f"yaw_error={yaw_error:.4f} linear_speed={linear_speed:.4f} "
        f"angular_speed={angular_speed:.4f} within_limit={within_limit}",
        flush=True,
    )
    return within_limit


def send_goal(node, client, share, frame, goal_data, options, tf_buffer, last_odom):
    mode, x, y, yaw = goal_data
    tree = Path(share) / "behavior_trees" / f"nav_{mode}.xml"
    if not tree.is_file():
        print(f"TREE_MISSING {tree}", flush=True)
        return False
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = frame
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
    goal.behavior_tree = str(tree)

    recoveries = [0]

    def on_feedback(message):
        count = message.feedback.number_of_recoveries
        if count != recoveries[0]:
            recoveries[0] = count
            print(f"RECOVERIES mode={mode} count={count}", flush=True)

    send_future = client.send_goal_async(goal, feedback_callback=on_feedback)
    if not spin_until(node, send_future, time.monotonic() + options.server_timeout):
        print(f"GOAL_RESPONSE_TIMEOUT mode={mode}", flush=True)
        return False
    handle = send_future.result()
    if not handle.accepted:
        print(f"GOAL_REJECTED mode={mode}", flush=True)
        return False
    print(f"GOAL_ACCEPTED mode={mode} behavior_tree={goal.behavior_tree}", flush=True)

    result_future = handle.get_result_async()
    start = time.monotonic()
    deadline = start + options.goal_timeout
    cancel_deadline = start + options.cancel_after if options.cancel_after else None
    while rclpy.ok() and not result_future.done():
        now = time.monotonic()
        if now >= deadline or (cancel_deadline is not None and now >= cancel_deadline):
            reason = "TIMEOUT" if now >= deadline else "REQUESTED"
            print(f"GOAL_CANCEL mode={mode} reason={reason}", flush=True)
            if not cancel_and_wait(node, handle, result_future, options.cancel_grace):
                return False
            break
        rclpy.spin_once(node, timeout_sec=min(0.1, deadline - now))

    if not result_future.done():
        print(f"RESULT_MISSING mode={mode}", flush=True)
        return False
    status = result_future.result().status
    print(
        f"GOAL_RESULT mode={mode} status={STATUS_NAMES.get(status, status)} "
        f"recoveries={recoveries[0]}", flush=True,
    )
    accuracy_ok = True
    if options.check_accuracy:
        # Keep the final pose and speed visible even when Nav2 aborts: they
        # help distinguish goal-checker tuning from localization/control drift.
        accuracy_ok = verify_accuracy(
            node, tf_buffer, last_odom, frame, goal_data, options
        )
    return (status == GoalStatus.STATUS_SUCCEEDED and
            cancel_deadline is None and accuracy_ok)


def main():
    options, goals = parse_args()
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("nav_action_probe")
    try:
        share = get_package_share_directory("go2_nav_bringup")
        client = ActionClient(node, NavigateToPose, options.action_name)
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
        last_odom = [None, 0.0]

        def on_odometry(message):
            last_odom[0] = message
            last_odom[1] = time.monotonic()

        odom_subscription = node.create_subscription(
            Odometry, "/odom", on_odometry, 10
        )
        if not options.skip_lifecycle_check and not wait_for_nav_active(
            node, options.lifecycle_service, options.server_timeout
        ):
            print("NAV_LIFECYCLE_TIMEOUT", flush=True)
            return 2
        if not client.wait_for_server(timeout_sec=options.server_timeout):
            print(f"ACTION_SERVER_TIMEOUT name={options.action_name}", flush=True)
            return 2
        for goal in goals:
            if not send_goal(
                node, client, share, options.frame, goal, options,
                tf_buffer, last_odom,
            ):
                return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
