"""Explicit SIMULATION ONLY arbiter/motion/relocalization services."""
import time
import rclpy
from go2_obstacle_interfaces.action import Relocalize
from go2_obstacle_interfaces.msg import ControlState
from go2_obstacle_interfaces.srv import GetMotionState, SetControlMode
from rclpy.action import ActionServer
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class MockMotionServices:
    """Test fixture only: never gates real cmd_vel or controls support/actuators."""
    def __init__(self, node):
        self.node = node
        self.owner, self.mission_id = "SAFE_HOLD", ""
        self.deadline = 0.0
        self.nav_active = self.skill_active = False
        self.stopped = self.safe_stance = self.releasable = True
        self.known = True
        self.fault = ""
        self.deny_owner = ""
        self.health = True
        self.relocalize_success = False
        self.on_relocalize = None
        self.stamp_provider = lambda: self.node.get_clock().now().to_msg()
        self.requests = []
        self.publisher = node.create_publisher(ControlState, "/motion/control_state", 10)
        self.switch_service = node.create_service(
            SetControlMode, "/motion/set_control_mode", self._switch)
        self.state_service = node.create_service(
            GetMotionState, "/motion/get_state", self._motion)
        self.action = ActionServer(
            node, Relocalize, "/localization/relocalize", self._relocalize)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = node.create_timer(0.05, self._publish, clock=self.clock)

    def _switch(self, req, res):
        self.requests.append(req.owner)
        safe = (req.owner == "SAFE_HOLD" or req.owner == self.owner or
                (not self.nav_active and not self.skill_active and self.stopped and
                 self.safe_stance and self.releasable and self.known and not self.fault))
        identity = (req.owner == "SAFE_HOLD" or
                    (req.expected_owner == self.owner and
                     (not self.mission_id or self.mission_id == req.mission_id)))
        accepted = (req.owner in ("NAVIGATION", "SKILL", "SAFE_HOLD") and
                    req.owner != self.deny_owner and safe and identity and
                    0.1 <= req.lease_duration <= 10.0)
        if accepted:
            self.owner, self.mission_id = req.owner, req.mission_id
            self.deadline = time.monotonic() + req.lease_duration
        res.accepted, res.owner, res.mission_id = accepted, self.owner, self.mission_id
        res.request_id = req.request_id
        res.message = "SIMULATION ONLY acknowledgment" if accepted else "mock guard rejected"
        self._publish()
        return res

    def _publish(self):
        if self.deadline and time.monotonic() >= self.deadline:
            self.owner, self.mission_id = "SAFE_HOLD", ""
            self.deadline = 0.0
        msg = ControlState(stamp=self.stamp_provider(),
                           owner=self.owner, mission_id=self.mission_id, healthy=self.health)
        self.publisher.publish(msg)

    def _motion(self, req, res):
        res.known, res.obstacle_id = self.known, req.obstacle_id
        res.skill_active, res.safe_stance = self.skill_active, self.safe_stance
        res.control_releasable, res.error_code = self.releasable, self.fault
        res.message = "SIMULATION ONLY motion state"
        return res

    def _relocalize(self, handle):
        result = Relocalize.Result(localized=self.relocalize_success,
                                   message="SIMULATION ONLY autonomous localization")
        if self.relocalize_success:
            if self.on_relocalize:
                self.on_relocalize()
            handle.succeed()
        else:
            result.error_code = "MOCK_NOT_READY"
            handle.abort()
        return result

    def destroy(self):
        self.timer.cancel()
        self.action.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = Node("mock_motion_services")
    node.declare_parameter("simulation_mode", False)
    if not node.get_parameter("simulation_mode").value:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("Mock motion services require simulation_mode=true")
    mock = MockMotionServices(node)
    node.get_logger().warning("SIMULATION ONLY: no hardware arbitration or safety support")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        mock.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

