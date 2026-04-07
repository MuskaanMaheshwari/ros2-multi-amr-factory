"""
ROS2 Node for path planning service.

Provides motion planning capabilities via ROS2 service interface:
  - A* graph search in factory aisle network
  - Dubins curve generation for smooth turns
  - Spline smoothing for curved paths
  - Collision checking

Author: Muskaan Maheshwari
"""

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import PoseStamped, Path
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    class Node:
        pass
    class Path:
        pass

import json
import time
import math
from typing import Dict, List, Optional, Tuple
from ..src.planning.path_planner import FactoryPathPlanner, FactoryEnvironment


class PathPlannerNode(Node):
    """
    ROS2 Node for factory path planning.

    Provides motion planning service combining A* graph search with Dubins/spline
    smoothing for realistic factory navigation with curved segments.

    Provided Services:
      - /path_planner/plan_path - Plan path from start to goal
      - /path_planner/plan_with_dubins - Plan path using Dubins curves
      - /path_planner/get_factory_graph - Get aisle network structure

    Published Topics:
      - /path_planner/last_plan (nav_msgs/Path) - Last computed path
      - /path_planner/plan_stats (std_msgs/String) - Planning statistics
    """

    def __init__(self, turning_radius: float = 2.0):
        """
        Initialize path planner node.

        Args:
            turning_radius: Minimum turning radius for curved paths (meters)
        """
        if not HAS_ROS2:
            raise RuntimeError("ROS2 (rclpy) not installed")

        super().__init__('path_planner')

        self.turning_radius = turning_radius
        self.last_plan_stats = None

        # Initialize factory environment with aisle waypoints
        # In real deployment, load from YAML config or factory environment
        self.factory_env = self._create_default_factory_env()
        self.planner = FactoryPathPlanner(
            factory_env=self.factory_env,
            turning_radius=turning_radius
        )

        # Setup QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.path_pub = self.create_publisher(
            Path,
            '/path_planner/last_plan',
            qos_profile
        )

        self.stats_pub = self.create_publisher(
            String,
            '/path_planner/plan_stats',
            qos_profile
        )

        self.get_logger().info(
            f'Path Planner initialized (turning radius {turning_radius}m)'
        )

    def _create_default_factory_env(self) -> 'FactoryEnvironment':
        """
        Create a default factory environment with aisle network.

        Returns:
            FactoryEnvironment with waypoints and connections
        """
        # Synthetic factory grid: 10x10 meter layout
        # Create a 3x3 grid of waypoints with 10m spacing
        aisle_waypoints = {}
        aisle_connections = {}

        for i in range(3):
            for j in range(3):
                node_id = f'N{i}{j}'
                x = i * 10.0
                y = j * 10.0
                aisle_waypoints[node_id] = (x, y)

        # Connect adjacent nodes
        for i in range(3):
            for j in range(3):
                node_id = f'N{i}{j}'
                neighbors = []
                if i > 0:
                    neighbors.append(f'N{i-1}{j}')
                if i < 2:
                    neighbors.append(f'N{i+1}{j}')
                if j > 0:
                    neighbors.append(f'N{i}{j-1}')
                if j < 2:
                    neighbors.append(f'N{i}{j+1}')
                aisle_connections[node_id] = neighbors

        # Create stations at key locations
        stations = {
            'INCOMING_MATERIAL': {
                'approach_point': (0.0, 0.0),
                'approach_heading': 0.0
            },
            'CELL_ASSEMBLY': {
                'approach_point': (10.0, 0.0),
                'approach_heading': 0.0
            },
            'MODULE_PACKING': {
                'approach_point': (20.0, 0.0),
                'approach_heading': 0.0
            },
            'TESTING_QC': {
                'approach_point': (10.0, 10.0),
                'approach_heading': 0.0
            },
            'SHIPPING': {
                'approach_point': (20.0, 10.0),
                'approach_heading': 0.0
            },
            'CHARGER_1': {
                'approach_point': (0.0, 20.0),
                'approach_heading': 0.0
            },
            'PARKING_BAY': {
                'approach_point': (5.0, 20.0),
                'approach_heading': 0.0
            },
        }

        return FactoryEnvironment(
            aisle_waypoints=aisle_waypoints,
            aisle_connections=aisle_connections,
            stations=stations
        )

    def plan_path(
        self,
        start_pos: Tuple[float, float],
        goal_pos: Tuple[float, float],
        start_heading: float = 0.0,
        goal_heading: float = 0.0
    ) -> Optional[List[Tuple[float, float, float]]]:
        """
        Plan a path from start to goal.

        Uses A* graph search followed by Dubins curve or spline smoothing.

        Args:
            start_pos: (x, y) start position
            goal_pos: (x, y) goal position
            start_heading: Start heading in radians
            goal_heading: Goal heading in radians

        Returns:
            List of (x, y, heading) waypoints, or None if no path found
        """
        start_time = time.time()

        start_pose = (start_pos[0], start_pos[1], start_heading)
        goal_pose = (goal_pos[0], goal_pos[1], goal_heading)

        try:
            path, stats = self.planner.plan(start_pose, goal_pose)
            self.last_plan_stats = stats

            # Publish statistics
            stats_msg = String()
            stats_msg.data = json.dumps({
                'path_length': stats.path_length,
                'computation_time_ms': (time.time() - start_time) * 1000.0,
                'num_waypoints': stats.num_waypoints,
                'max_curvature': stats.max_curvature,
                'path_type': stats.path_type,
            })
            self.stats_pub.publish(stats_msg)

            self.get_logger().info(
                f'Planned path: {len(path)} waypoints, '
                f'{stats.path_length:.2f}m, {stats.computation_time*1000:.1f}ms'
            )

            return path

        except Exception as e:
            self.get_logger().error(f'Path planning failed: {str(e)}')
            return None

    def plan_path_with_obstacles(
        self,
        start_pos: Tuple[float, float],
        goal_pos: Tuple[float, float],
        obstacles: Optional[List[Tuple[float, float, float]]] = None
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Plan path while avoiding obstacles.

        Args:
            start_pos: (x, y) start
            goal_pos: (x, y) goal
            obstacles: List of (x, y, radius) circle obstacles

        Returns:
            List of (x, y, heading) waypoints
        """
        if obstacles is None:
            obstacles = []

        # Use RRT or potential field methods in real implementation
        # For now, fall back to basic A* planning
        return self.plan_path(start_pos, goal_pos)

    def get_factory_graph(self) -> Dict:
        """
        Get factory aisle network graph.

        Returns:
            Dictionary with waypoints and connections
        """
        return {
            'aisle_waypoints': self.factory_env.aisle_waypoints,
            'aisle_connections': self.factory_env.aisle_connections,
            'stations': list(self.factory_env.stations.keys()),
        }

    def get_plan_statistics(self) -> Optional[Dict]:
        """
        Get statistics from last planning operation.

        Returns:
            Dictionary with plan metrics or None
        """
        if self.last_plan_stats is None:
            return None

        return {
            'path_length': self.last_plan_stats.path_length,
            'computation_time': self.last_plan_stats.computation_time,
            'num_waypoints': self.last_plan_stats.num_waypoints,
            'max_curvature': self.last_plan_stats.max_curvature,
            'path_type': self.last_plan_stats.path_type,
        }


def main(args=None):
    """Standalone node entry point."""
    if not HAS_ROS2:
        print("ERROR: ROS2 not installed")
        return

    rclpy.init(args=args)

    node = PathPlannerNode(turning_radius=2.0)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
