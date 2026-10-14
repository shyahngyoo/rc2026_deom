"""Autonomous localization Action; absence is explicitly not ready."""
from go2_obstacle_interfaces.action import Relocalize
from .navigation_client import AsyncActionRunner


class RelocalizationClient(AsyncActionRunner):
    def __init__(self, node, action_name, frame, server_timeout, cancel_timeout):
        super().__init__(node, Relocalize, action_name, server_timeout, cancel_timeout)
        self.frame = frame

    def execute(self, command, done):
        self.start(command.token, Relocalize.Goal(frame_id=self.frame),
                   command.timeout, done)

