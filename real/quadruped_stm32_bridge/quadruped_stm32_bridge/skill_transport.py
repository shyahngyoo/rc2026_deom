"""Reuse Hurricane's gateway transport without a second serial owner.

The /execute_obstacle ActionServer remains in robocon_mission.  This package
only exposes the integration type; it does not start another ActionServer.
"""
from robocon_mission.hurricane_gateway_transport import GatewayTransport

__all__ = ["GatewayTransport"]
