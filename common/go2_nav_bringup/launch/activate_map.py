"""Bounded map lifecycle startup, including recovery from lost service replies.

Only this process owns map transitions. Nav2's navigation lifecycle manager
continues to own the navigation nodes and their runtime bonds.
"""
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class MapActivator:
    def __init__(self, node, timeout, request_timeout=5.0):
        self.node = node
        self.deadline = time.monotonic() + timeout
        self.request_timeout = request_timeout
        self.clients = {}
        self.retryable = False

    def call(self, kind, request):
        service_type = GetState if kind == "get_state" else ChangeState
        client = self.clients.get(kind)
        if client is None:
            client = self.node.create_client(service_type, "/map_server/" + kind)
            self.clients[kind] = client
        remaining = self.deadline - time.monotonic()
        if remaining <= 0 or not client.wait_for_service(timeout_sec=min(1.0, remaining)):
            return None
        future = client.call_async(request)
        until = min(self.deadline, time.monotonic() + self.request_timeout)
        while rclpy.ok() and not future.done() and time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=min(0.1, max(0.0, until - time.monotonic())))
        if future.done():
            try:
                return future.result()
            except Exception as error:
                self.node.get_logger().warning(f"map_server/{kind}: {error}")
        else:
            self.node.get_logger().warning(
                f"map_server/{kind} reply timed out; recreating client and checking actual state"
            )
        # A fresh client also rebuilds DDS request/reply discovery after a
        # timeout. Do not leave orphaned requests or retry a transition blindly.
        client.remove_pending_request(future)
        self.node.destroy_client(client)
        self.clients.pop(kind, None)
        return None

    def run(self):
        rejected_state = None
        last_state = "unavailable"
        while rclpy.ok() and time.monotonic() < self.deadline:
            response = self.call("get_state", GetState.Request())
            if response is None:
                continue
            state = response.current_state.id
            last_state = response.current_state.label
            if state == State.PRIMARY_STATE_ACTIVE:
                print("READY stage=map_activate state=active", flush=True)
                return 0
            if state == State.PRIMARY_STATE_FINALIZED:
                break
            if state == rejected_state:
                # A delivered rejection with no state change is a real failure
                # (e.g. invalid map YAML), not a lost acknowledgement.
                self.node.get_logger().error(f"Map transition rejected; state={last_state}")
                return 1
            transition = {
                State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
                State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE,
            }.get(state)
            if transition is None:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                continue
            request = ChangeState.Request()
            request.transition.id = transition
            result = self.call("change_state", request)
            rejected_state = state if result is not None and not result.success else None
            # Always query the actual state before the next transition. A
            # timeout can mean that configuration succeeded but its reply was lost.
        self.node.get_logger().error(f"Map activation timed out or stopped; last_state={last_state}")
        self.retryable = time.monotonic() >= self.deadline
        return 1


def activate_map(timeout):
    """Rebuild the DDS participant if client recreation alone cannot recover."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.init(args=[])
        node = Node("go2_nav_wait_map_activate")
        try:
            activator = MapActivator(node, min(20.0, deadline - time.monotonic()))
            result = activator.run()
            retryable = activator.retryable
        finally:
            node.destroy_node()
            rclpy.shutdown()
        if result == 0 or not retryable:
            return result
        print("Map communication recovery: rebuilding DDS context within startup deadline", flush=True)
    return 1
