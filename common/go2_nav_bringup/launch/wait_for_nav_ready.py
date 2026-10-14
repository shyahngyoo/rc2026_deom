#!/usr/bin/python3
"""Wait for actual map/TF/Nav2 readiness before starting dependent launch actions."""

import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

GET_STATE_POLL_INTERVAL = 0.75
GET_STATE_RESPONSE_TIMEOUT = 5.0
ICP_POSE_REQUIRED_COUNT = 3
ICP_POSE_MAX_GAP = 4.0


class AcceptedPoseSequence:
    def __init__(self):
        self.count = 0
        self.last_stamp = None
        self.last_received = None

    def observe(self, message):
        received = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if message.header.frame_id != "map" or stamp <= 0:
            self.count = 0
            self.last_stamp = None
            self.last_received = None
            return
        if self.last_stamp is not None:
            stamp_gap = stamp - self.last_stamp
            received_gap = received - self.last_received
            if stamp_gap <= 0:
                self.count = 0
                self.last_stamp = None
                self.last_received = None
                return
            if stamp_gap > ICP_POSE_MAX_GAP or received_gap > ICP_POSE_MAX_GAP:
                self.count = 0
        self.count += 1
        self.last_stamp = stamp
        self.last_received = received

    def ready(self, now):
        return (
            self.count >= ICP_POSE_REQUIRED_COUNT
            and self.last_received is not None
            and now - self.last_received <= ICP_POSE_MAX_GAP
        )


def main():
    stage = sys.argv[1]
    timeout = float(sys.argv[2])
    if stage == "map_activate":
        from activate_map import activate_map
        return activate_map(timeout)
    rclpy.init(args=[])
    node = Node("go2_nav_wait_" + stage)
    names = {
        "map_service": ("map_server",),
        "map_tf": ("map_server",),
        "nav_action": ("bt_navigator", "planner_server", "controller_server"),
    }[stage]
    clients = {
        name: node.create_client(GetState, "/" + name + "/get_state")
        for name in names
    }
    futures = {}
    pending_since = {}
    next_query_at = {name: 0.0 for name in names}
    states = {name: "unavailable" for name in names}
    buffer = Buffer() if stage == "map_tf" else None
    listener = TransformListener(buffer, node) if buffer is not None else None
    poses = AcceptedPoseSequence()
    pose_subscription = (
        node.create_subscription(
            PoseWithCovarianceStamped, "/localization/pose",
            poses.observe, 10
        ) if stage == "map_tf" else None
    )
    action = (
        ActionClient(node, NavigateToPose, "/navigate_to_pose")
        if stage == "nav_action" else None
    )
    started = time.monotonic()
    try:
        while time.monotonic() - started < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            for name, client in clients.items():
                future = futures.get(name)
                if (
                    future is not None
                    and not future.done()
                    and now - pending_since[name] >= GET_STATE_RESPONSE_TIMEOUT
                ):
                    client.remove_pending_request(future)
                    futures.pop(name, None)
                    states[name] = "response_timeout"
                    print(
                        f"GetState response timeout node={name} "
                        f"after={now - pending_since[name]:.1f}s; retrying",
                        file=sys.stderr,
                        flush=True,
                    )
                    future = None
                if future is not None and future.done():
                    try:
                        states[name] = future.result().current_state.label
                    except Exception:
                        states[name] = "error"
                    futures.pop(name, None)
                if (
                    name not in futures
                    and now >= next_query_at[name]
                    and client.service_is_ready()
                ):
                    futures[name] = client.call_async(GetState.Request())
                    pending_since[name] = now
                    next_query_at[name] = now + GET_STATE_POLL_INTERVAL

            if stage == "map_service":
                ready = states["map_server"] not in (
                    "unavailable", "error", "response_timeout"
                )
            elif stage == "map_tf":
                ready = (
                    states["map_server"] == "active"
                    and poses.ready(now)
                    and buffer.can_transform("map", "odom", Time())
                )
            else:
                ready = (
                    all(state == "active" for state in states.values()) and
                    action.server_is_ready()
                )
            if ready:
                print(
                    f"READY stage={stage} elapsed={time.monotonic()-started:.1f}s "
                    f"states={states}",
                    flush=True,
                )
                return 0

        print(
            f"NOT_READY stage={stage} timeout={timeout:.0f}s states={states}",
            file=sys.stderr,
            flush=True,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

