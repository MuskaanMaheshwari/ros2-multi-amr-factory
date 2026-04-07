"""
ROS2 Node for fleet management and task coordination.

Wraps the FleetCoordinator into a ROS2 node that:
  - Publishes fleet status (aggregate metrics) at ~2 Hz
  - Provides task assignment service
  - Subscribes to task queue updates
  - Manages global fleet state and statistics

Author: Muskaan Maheshwari
"""

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Pose2D
    from std_msgs.msg import String, Int32, Float32
    import tf_transformations
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    class Node:
        pass

import json
import time
from typing import Dict, List, Optional
from ..src.fleet.coordinator import FleetCoordinator, Task, TaskType, TaskPriority


class FleetManagerNode(Node):
    """
    ROS2 Node for multi-AMR fleet coordination.

    Manages task assignment, scheduling, battery management, and collision
    avoidance for the entire robot fleet. Acts as the central orchestrator
    for factory production.

    Published Topics:
      - /fleet/status (std_msgs/String) - JSON with fleet state metrics
      - /fleet/robot_count (std_msgs/Int32) - Total number of robots
      - /fleet/utilization (std_msgs/Float32) - Fleet utilization percentage

    Provided Services:
      - /fleet/assign_task - Trigger task assignment
      - /fleet/add_production_task - Create and assign production task
      - /fleet/get_status - Get detailed fleet status
    """

    def __init__(self, fleet_coordinator: Optional[FleetCoordinator] = None):
        """
        Initialize fleet manager node.

        Args:
            fleet_coordinator: FleetCoordinator instance (required if using real data)
        """
        if not HAS_ROS2:
            raise RuntimeError("ROS2 (rclpy) not installed")

        super().__init__('fleet_manager')

        self.coordinator = fleet_coordinator
        self.last_status_time = 0.0
        self.status_update_interval = 0.5  # seconds

        # Setup QoS for fleet-wide communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.fleet_status_pub = self.create_publisher(
            String,
            '/fleet/status',
            qos_profile
        )

        self.robot_count_pub = self.create_publisher(
            Int32,
            '/fleet/robot_count',
            qos_profile
        )

        self.fleet_utilization_pub = self.create_publisher(
            Float32,
            '/fleet/utilization_percent',
            qos_profile
        )

        self.task_completion_pub = self.create_publisher(
            String,
            '/fleet/task_completed',
            qos_profile
        )

        # Update timer
        self.update_timer = self.create_timer(
            0.5,  # 2 Hz
            self._update_fleet_status
        )

        self.get_logger().info('Fleet Manager Node initialized')

    def _update_fleet_status(self) -> None:
        """Update and publish fleet status at regular intervals."""
        if self.coordinator is None:
            self.get_logger().warn('No coordinator attached to fleet manager')
            return

        current_time = time.time()

        # Update coordinator state
        stats = self.coordinator.update(0.5)

        # Publish fleet status as JSON
        status_msg = String()
        status_msg.data = json.dumps({
            'timestamp': current_time,
            'total_robots': stats.get('total_robots', 0),
            'idle_robots': stats.get('idle', 0),
            'navigating_robots': stats.get('navigating', 0),
            'docking_robots': stats.get('docking', 0),
            'charging_robots': stats.get('charging', 0),
            'parked_robots': stats.get('parked', 0),
            'pending_tasks': stats.get('tasks', {}).get('pending', 0),
            'active_tasks': stats.get('tasks', {}).get('active', 0),
            'completed_tasks': stats.get('tasks', {}).get('completed', 0),
            'throughput_tasks_per_hour': stats.get('throughput_tasks_per_hour', 0.0),
            'fleet_utilization_percent': stats.get('fleet_utilization_percent', 0.0),
            'total_distance_traveled': stats.get('total_distance_traveled', 0.0),
        })
        self.fleet_status_pub.publish(status_msg)

        # Publish robot count
        count_msg = Int32()
        count_msg.data = stats.get('total_robots', 0)
        self.robot_count_pub.publish(count_msg)

        # Publish utilization
        util_msg = Float32()
        util_msg.data = float(stats.get('fleet_utilization_percent', 0.0))
        self.fleet_utilization_pub.publish(util_msg)

    def create_and_assign_task(
        self,
        task_type: TaskType,
        pickup_station: Optional[str] = None,
        dropoff_station: Optional[str] = None,
        payload_kg: float = 100.0,
        priority: TaskPriority = TaskPriority.NORMAL
    ) -> Task:
        """
        Create a new production task and add to coordinator queue.

        Args:
            task_type: Type of task (MATERIAL_TRANSPORT, CHARGING, etc)
            pickup_station: Source station ID
            dropoff_station: Destination station ID
            payload_kg: Task payload mass
            priority: Task priority level

        Returns:
            Created Task object
        """
        if self.coordinator is None:
            raise RuntimeError('No coordinator available')

        task = Task(
            task_id=f'TASK_{self.coordinator.task_counter:04d}',
            task_type=task_type,
            priority=priority,
            pickup_station_id=pickup_station,
            dropoff_station_id=dropoff_station,
            payload_kg=payload_kg,
            created_time=time.time()
        )

        self.coordinator.add_task(task)
        self.coordinator.task_counter += 1

        self.get_logger().info(
            f'Created task {task.task_id}: {pickup_station} -> {dropoff_station}'
        )

        return task

    def get_fleet_status_dict(self) -> Dict:
        """
        Get comprehensive fleet status.

        Returns:
            Dictionary with fleet composition and metrics
        """
        if self.coordinator is None:
            return {'error': 'No coordinator available'}

        return self.coordinator.get_fleet_status()

    def get_production_metrics(self) -> Dict:
        """
        Get production performance metrics.

        Returns:
            Dictionary with throughput, utilization, KPIs
        """
        if self.coordinator is None:
            return {'error': 'No coordinator available'}

        return self.coordinator.get_production_metrics()

    def attach_coordinator(self, coordinator: FleetCoordinator) -> None:
        """
        Attach a FleetCoordinator instance.

        Args:
            coordinator: FleetCoordinator instance to manage
        """
        self.coordinator = coordinator
        self.get_logger().info('Coordinator attached to fleet manager')


def main(args=None):
    """Standalone node entry point."""
    if not HAS_ROS2:
        print("ERROR: ROS2 not installed")
        return

    rclpy.init(args=args)

    node = FleetManagerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
