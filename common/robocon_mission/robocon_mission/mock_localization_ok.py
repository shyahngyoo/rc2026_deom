"""Simulation-only positive localization health service for safe test routes."""

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node("mock_localization_ok")

    def respond(_request, response):
        response.success = True
        response.message = "SIMULATION ONLY: health approval is mocked"
        return response

    service = node.create_service(Trigger, "/mission/localization_ok", respond)
    node.get_logger().warn("Mock localization health: simulation testing only")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_service(service)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
