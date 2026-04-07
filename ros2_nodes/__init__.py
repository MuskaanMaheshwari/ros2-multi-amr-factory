"""
ROS2 Node wrappers for multi-AMR factory system.

This package provides ROS2 node implementations that wrap the core simulation
components (robot physics, fleet coordination, traffic management, path planning)
into a ROS2-compliant architecture.

Nodes:
  - amr_driver_node: Wraps individual AMR robot physics
  - fleet_manager_node: Wraps fleet coordinator for task assignment
  - traffic_controller_node: Wraps traffic manager for intersection control
  - path_planner_node: Provides path planning service

Author: Muskaan Maheshwari
"""

__all__ = [
    "amr_driver_node",
    "fleet_manager_node",
    "traffic_controller_node",
    "path_planner_node",
]
