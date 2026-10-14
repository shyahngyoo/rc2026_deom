"""Versioned JSON status schema carried by std_msgs/String."""

import json

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MissionStatusPublisher:
    def __init__(self, node, topic="/mission/status"):
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = node.create_publisher(String, topic, qos)

    def publish(self, fsm, feedback="", control_owner="UNKNOWN",
                remaining_time=None, end_errors=None, waypoint_errors=None):
        task = fsm.route.current
        payload = {
            "schema_version": 1,
            "state": fsm.state.value,
            "task_id": task.id if task else "",
            "task_type": task.type if task else "",
            "task_index": fsm.route.index,
            "task_total": len(fsm.route.tasks),
            "completed_task_ids": list(fsm.route.completed),
            "error": fsm.error,
            "navigation_mode": fsm.nav_mode,
            "finished": fsm.state.value in ("FINISHED", "ABORTED"),
            "feedback": feedback,
            "command": fsm.pending.kind if fsm.pending else "",
            "token": fsm.pending.token if fsm.pending else fsm.sequence,
            "control_owner": control_owner,
            "remaining_time": remaining_time,
            "end_errors": end_errors,
            "waypoint_errors": waypoint_errors,
            "waypoint_alignment_attempts": fsm.waypoint_alignment_attempts,
            "obstacle_physical_verified": fsm.end_verified,
            "rule_validation": "not_implemented",
            "rule_verdicts": {key: {"scored": value.scored, "reason": value.reason}
                              for key, value in fsm.rule_verdicts.items()},
        }
        self.publisher.publish(String(data=json.dumps(payload, sort_keys=True)))
