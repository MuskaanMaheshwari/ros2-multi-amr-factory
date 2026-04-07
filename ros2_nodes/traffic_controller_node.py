"""
ROS2 Node for traffic control and intersection management.

Wraps the TrafficManager into a ROS2 node that:
  - Manages intersection states (FREE, CLAIMED, OCCUPIED)
  - Publishes intersection status updates
  - Provides services for robot reservation at intersections
  - Detects and resolves deadlocks
  - Implements priority-based right-of-way

Author: Muskaan Maheshwari
"""

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String, Int32
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    class Node:
        pass

import json
import time
from typing import Dict, List, Optional, Tuple
from ..src.traffic.traffic_manager import TrafficManager, Intersection, IntersectionState


class TrafficControllerNode(Node):
    """
    ROS2 Node for traffic coordination in factory.

    Manages multi-robot traffic flows at intersections, prevents collisions,
    enforces priority-based right-of-way, and resolves deadlocks.

    Published Topics:
      - /traffic/intersection_states (std_msgs/String) - JSON with all intersections
      - /traffic/deadlock_alert (std_msgs/String) - Deadlock detection/resolution
      - /traffic/robot_priority[robot_id] (std_msgs/Int32) - Robot priority level

    Parameters:
      - num_intersections: Number of intersections in factory grid
      - intersection_radius: Zone of control for each intersection (meters)
      - safety_radius: Minimum safe distance between robots (meters)
    """

    def __init__(
        self,
        num_intersections: int = 9,
        intersection_radius: float = 3.0,
        safety_radius: float = 1.5
    ):
        """
        Initialize traffic controller node.

        Args:
            num_intersections: Number of intersections to manage
            intersection_radius: Zone radius for each intersection
            safety_radius: Minimum separation between robots
        """
        if not HAS_ROS2:
            raise RuntimeError("ROS2 (rclpy) not installed")

        super().__init__('traffic_controller')

        self.num_intersections = num_intersections
        self.intersection_radius = intersection_radius
        self.safety_radius = safety_radius

        # Create synthetic intersection grid (3x3)
        # In real deployment, load from factory environment
        grid_spacing = 30.0  # meters between intersections
        intersections = []
        for i in range(3):
            for j in range(3):
                x = i * grid_spacing
                y = j * grid_spacing
                intersections.append((x, y))

        self.traffic_manager = TrafficManager(
            intersections=intersections,
            safety_radius=safety_radius
        )

        # Update intersection radii
        for intersection in self.traffic_manager.intersections.values():
            intersection.radius = intersection_radius

        # Setup QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.intersection_states_pub = self.create_publisher(
            String,
            '/traffic/intersection_states',
            qos_profile
        )

        self.deadlock_alert_pub = self.create_publisher(
            String,
            '/traffic/deadlock_alert',
            qos_profile
        )

        # Update timer for traffic state
        self.update_timer = self.create_timer(
            0.1,  # 10 Hz
            self._update_traffic_state
        )

        self.get_logger().info(
            f'Traffic Controller initialized with {len(self.traffic_manager.intersections)} '
            f'intersections (radius {intersection_radius}m, safety {safety_radius}m)'
        )

    def _update_traffic_state(self) -> None:
        """Update and publish traffic state at regular intervals."""
        current_time = time.time()

        # Update traffic manager (would call real update in integrated system)
        self.traffic_manager.update(0.1)

        # Publish intersection states
        intersection_states = {}
        for int_id, intersection in self.traffic_manager.intersections.items():
            intersection_states[int_id] = {
                'center': intersection.center,
                'state': IntersectionState(intersection.state).name,
                'claimed_by': intersection.claimed_by,
                'queue_length': len(intersection.queue),
                'queue': intersection.queue,
            }

        status_msg = String()
        status_msg.data = json.dumps({
            'timestamp': current_time,
            'intersections': intersection_states,
            'active_deadlocks': len(self.traffic_manager.active_deadlocks)
        })
        self.intersection_states_pub.publish(status_msg)

        # Publish deadlock alerts if any detected
        if self.traffic_manager.active_deadlocks:
            alert_msg = String()
            deadlocks_data = {}
            for deadlock_id, info in self.traffic_manager.active_deadlocks.items():
                deadlocks_data[deadlock_id] = {
                    'robot_ids': info.robot_ids,
                    'intersection_ids': info.intersection_ids,
                    'detected_time': info.detected_time,
                    'resolved': info.resolved,
                    'resolution_method': info.resolution_method,
                }
            alert_msg.data = json.dumps({
                'timestamp': current_time,
                'deadlocks': deadlocks_data
            })
            self.deadlock_alert_pub.publish(alert_msg)

    def reserve_intersection(
        self,
        robot_id: str,
        intersection_id: str,
        priority_level: int = 2
    ) -> bool:
        """
        Request reservation of an intersection.

        Args:
            robot_id: ID of requesting robot
            intersection_id: ID of target intersection
            priority_level: Priority (0=emergency, 1=high, 2=normal, 3=low)

        Returns:
            True if reservation successful
        """
        intersection = self.traffic_manager.intersections.get(intersection_id)
        if intersection is None:
            self.get_logger().warn(f'Unknown intersection: {intersection_id}')
            return False

        if intersection.state == IntersectionState.FREE:
            intersection.state = IntersectionState.CLAIMED
            intersection.claimed_by = robot_id
            self.get_logger().debug(f'{robot_id} reserved {intersection_id}')
            return True
        elif intersection.claimed_by == robot_id:
            return True  # Already claimed by this robot
        else:
            # Add to queue with priority
            if robot_id not in intersection.queue:
                intersection.queue.append(robot_id)
            return False

    def release_intersection(self, robot_id: str, intersection_id: str) -> bool:
        """
        Release a reserved intersection.

        Args:
            robot_id: ID of robot releasing
            intersection_id: ID of intersection to release

        Returns:
            True if successfully released
        """
        intersection = self.traffic_manager.intersections.get(intersection_id)
        if intersection is None:
            return False

        if intersection.claimed_by == robot_id:
            intersection.state = IntersectionState.FREE
            intersection.claimed_by = None
            intersection.last_clear_time = time.time()
            self.get_logger().debug(f'{robot_id} released {intersection_id}')
            return True

        return False

    def get_next_robot_in_queue(self, intersection_id: str) -> Optional[str]:
        """
        Get the next robot waiting at an intersection.

        Args:
            intersection_id: ID of intersection

        Returns:
            Robot ID of next in queue, or None if queue empty
        """
        intersection = self.traffic_manager.intersections.get(intersection_id)
        if intersection and len(intersection.queue) > 0:
            return intersection.queue[0]
        return None

    def get_traffic_state_dict(self) -> Dict:
        """
        Get complete traffic state.

        Returns:
            Dictionary with intersection states and deadlock info
        """
        state = {
            'intersections': {},
            'deadlocks': {}
        }

        for int_id, intersection in self.traffic_manager.intersections.items():
            state['intersections'][int_id] = {
                'center': intersection.center,
                'state': IntersectionState(intersection.state).name,
                'claimed_by': intersection.claimed_by,
                'queue': intersection.queue,
            }

        for dl_id, info in self.traffic_manager.active_deadlocks.items():
            state['deadlocks'][dl_id] = {
                'robot_ids': info.robot_ids,
                'intersection_ids': info.intersection_ids,
                'resolved': info.resolved,
                'resolution_method': info.resolution_method,
            }

        return state

    def attach_traffic_manager(self, traffic_manager: TrafficManager) -> None:
        """
        Attach an existing TrafficManager instance.

        Args:
            traffic_manager: TrafficManager to manage
        """
        self.traffic_manager = traffic_manager
        self.get_logger().info('TrafficManager attached')


def main(args=None):
    """Standalone node entry point."""
    if not HAS_ROS2:
        print("ERROR: ROS2 not installed")
        return

    rclpy.init(args=args)

    node = TrafficControllerNode(num_intersections=9, intersection_radius=3.0)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
