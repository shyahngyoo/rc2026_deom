"""Exercise NavigateToPose client logic against an in-process fake Nav2 server.

This does not start Nav2, load a BT, or test the future obstacle Skill Action.
It uses the existing DDS configuration without environment overrides.
"""

import argparse
import runpy
import threading
import time
from pathlib import Path

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


SOURCE = Path(__file__).resolve().parents[1]
SEND_GOAL = runpy.run_path(str(SOURCE / "scripts" / "nav_action_probe.py"))["send_goal"]


class FakeNavigator(Node):
    def __init__(self, scenario):
        super().__init__("fake_nav_to_pose_client_test")
        self.scenario = scenario
        self.goals = []
        self.canceled = threading.Event()
        self.server = ActionServer(
            self, NavigateToPose, "/test_navigate_to_pose", self.execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

    def execute(self, handle):
        self.goals.append(handle.request)
        if self.scenario == "abort":
            handle.abort()
        elif self.scenario == "cancel":
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    handle.canceled()
                    self.canceled.set()
                    return NavigateToPose.Result()
                time.sleep(0.02)
            handle.abort()
        else:
            time.sleep(0.1)
            handle.succeed()
        return NavigateToPose.Result()


@pytest.mark.parametrize("scenario,expected_count", [
    ("success", 2), ("abort", 1), ("cancel", 1),
])
def test_transit_then_approach(scenario, expected_count, capsys):
    rclpy.init()
    server = FakeNavigator(scenario)
    client_node = rclpy.create_node("nav_to_pose_client_test")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        client = ActionClient(
            client_node, NavigateToPose, "/test_navigate_to_pose"
        )
        assert client.wait_for_server(timeout_sec=8.0)
        options = argparse.Namespace(
            server_timeout=8.0, goal_timeout=5.0, cancel_grace=5.0,
            cancel_after=0.3 if scenario == "cancel" else None,
            check_accuracy=False,
        )
        share = get_package_share_directory("go2_nav_bringup")
        goals = [
            ("transit", 1.0, 0.0, 0.0),
            ("approach", 1.5, 0.0, 0.2),
        ]
        results = []
        for goal in goals:
            result = SEND_GOAL(
                client_node, client, share, "map", goal, options, None, None
            )
            results.append(result)
            if not result:
                break

        output = capsys.readouterr().out
        assert results == ([True, True] if scenario == "success" else [False])
        assert len(server.goals) == expected_count
        share_path = Path(share)
        assert server.goals[0].behavior_tree == str(
            share_path / "behavior_trees" / "nav_transit.xml"
        )
        assert server.goals[0].pose.pose.position.x == 1.0
        assert server.goals[0].pose.header.frame_id == "map"
        if scenario == "success":
            assert server.goals[1].behavior_tree == str(
                share_path / "behavior_trees" / "nav_approach.xml"
            )
            assert server.goals[1].pose.pose.position.x == 1.5
            assert "GOAL_RESULT mode=transit status=SUCCEEDED" in output
            assert "GOAL_RESULT mode=approach status=SUCCEEDED" in output
        if scenario == "cancel":
            assert server.canceled.wait(2)
            assert "GOAL_RESULT mode=transit status=CANCELED" in output
        if scenario == "abort":
            assert "GOAL_RESULT mode=transit status=ABORTED" in output
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        server.server.destroy()
        server.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
