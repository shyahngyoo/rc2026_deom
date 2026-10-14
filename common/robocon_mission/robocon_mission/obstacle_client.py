"""ExecuteObstacle adapter; real motion handoff is deliberately separate."""

from go2_obstacle_interfaces.action import ExecuteObstacle
from typing import Callable

from .navigation_client import AsyncActionRunner
from .mission_fsm import Command


class ObstacleClient(AsyncActionRunner):
    def __init__(self, node, action_name, server_timeout, cancel_timeout):
        super().__init__(node, ExecuteObstacle, action_name, server_timeout,
                         cancel_timeout)

    def execute(self, command: Command, done: Callable) -> None:
        # Joint trajectories remain entirely inside Motion Manager.
        # Extended result safety evidence is checked by FSM and independent state polling.
        goal = ExecuteObstacle.Goal(
            obstacle_id=command.task_id, skill_id=command.skill_id
        )
        self.start(command.token, goal, command.timeout, done)
