"""
ROS2 Node for individual AMR robot driver.

Wraps the physics-based AMRRobot model into a ROS2 node that:
  - Subscribes to cmd_vel (geometry_msgs/Twist) for velocity commands
  - Subscribes to turret_cmd for independent turret rotation
  - Publishes odometry (nav_msgs/Odometry) at ~10 Hz
  - Publishes battery state (sensor_msgs/BatteryState)
  - Publishes robot status (custom message or std_msgs)
  - Provides emergency stop service

Author: Muskaan Maheshwari
"""

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Twist, Pose, Quaternion, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Float64, Header
    from std_srvs.srv import Empty, Trigger
    import tf_transformations
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    # Stub classes for when ROS2 is not available
    class Node:
        pass
    class Twist:
        pass
    class Odometry:
        pass
    class BatteryState:
        pass

import math
import time
from typing import Optional
from ..src.amr.robot import AMRRobot, AMRState


class AMRDriverNode(Node):
    """
    ROS2 Node for individual AMR robot.

    Manages the physics simulation of one AMR robot, interfaces with ROS2 pub/sub,
    and provides services for robot control and emergency stop.

    Parameters (via launch file or rosparam):
      - robot_id: str - Robot identifier (e.g., 'amr_001')
      - start_x, start_y: float - Initial position in meters
      - start_heading: float - Initial heading in radians
      - update_rate: int - Update frequency in Hz (default 10)
    """

    def __init__(self, robot_id: str, start_x: float = 0.0, start_y: float = 0.0,
                 start_heading: float = 0.0, update_rate: int = 10):
        """
        Initialize the AMR driver node.

        Args:
            robot_id: Unique robot identifier
            start_x: Initial X position (meters)
            start_y: Initial Y position (meters)
            start_heading: Initial heading (radians)
            update_rate: Update frequency in Hz
        """
        if not HAS_ROS2:
            raise RuntimeError("ROS2 (rclpy) not installed. Install via: pip install rclpy")

        super().__init__(f'{robot_id}_driver')

        self.robot_id = robot_id
        self.update_rate_hz = update_rate
        self.dt = 1.0 / update_rate

        # Initialize physics model
        self.robot = AMRRobot(
            robot_id=robot_id,
            start_position=(start_x, start_y),
            start_heading=start_heading
        )

        # Store latest velocity commands
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        self.turret_target_heading = 0.0

        # Setup QoS policy for real-time performance
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers: velocity command and turret command
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            f'/{robot_id}/cmd_vel',
            self._handle_cmd_vel,
            qos_profile
        )

        self.turret_cmd_sub = self.create_subscription(
            Float64,
            f'/{robot_id}/turret_cmd',
            self._handle_turret_cmd,
            qos_profile
        )

        # Publishers: odometry, battery state, robot status
        self.odom_pub = self.create_publisher(
            Odometry,
            f'/{robot_id}/odom',
            qos_profile
        )

        self.battery_pub = self.create_publisher(
            BatteryState,
            f'/{robot_id}/battery',
            qos_profile
        )

        self.status_pub = self.create_publisher(
            Float64,  # Placeholder: would be custom RobotStatus message
            f'/{robot_id}/status',
            qos_profile
        )

        # Services: emergency stop and reset
        self.emergency_stop_srv = self.create_service(
            Empty,
            f'/{robot_id}/emergency_stop',
            self._handle_emergency_stop
        )

        self.reset_srv = self.create_service(
            Trigger,
            f'/{robot_id}/reset',
            self._handle_reset
        )

        # Main update timer
        self.update_timer = self.create_timer(
            self.dt,
            self._update_step
        )

        self.get_logger().info(
            f'Initialized AMR driver node for {robot_id} at ({start_x:.2f}, {start_y:.2f})'
        )

    def _handle_cmd_vel(self, msg: Twist) -> None:
        """
        Callback for velocity command.

        Args:
            msg: geometry_msgs/Twist with linear.x (m/s) and angular.z (rad/s)
        """
        self.last_linear_vel = msg.linear.x
        self.last_angular_vel = msg.angular.z
        self.robot.set_velocity(msg.linear.x, msg.angular.z)

    def _handle_turret_cmd(self, msg: Float64) -> None:
        """
        Callback for turret command.

        Args:
            msg: std_msgs/Float64 containing target heading in radians
        """
        self.turret_target_heading = msg.data

    def _handle_emergency_stop(self, request, response) -> None:
        """
        Emergency stop service handler.
        Immediately halts all motion and sets emergency state.
        """
        self.robot.emergency_stop()
        self.get_logger().warn(f'{self.robot_id}: EMERGENCY STOP activated')
        return response

    def _handle_reset(self, request, response) -> None:
        """
        Reset robot to idle state.
        """
        self.robot.state = AMRState.IDLE
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        response.success = True
        response.message = f'{self.robot_id} reset to idle'
        return response

    def _update_step(self) -> None:
        """
        Main physics and sensor update step.
        Called at ~10 Hz (configurable).
        """
        # Update physics
        self.robot.update(self.dt)

        # Update turret if needed
        if abs(self.turret_target_heading - self.robot.turret_heading) > 0.01:
            self.robot.rotate_turret(self.turret_target_heading, self.dt)

        # Publish odometry
        self._publish_odometry()

        # Publish battery state
        self._publish_battery()

    def _publish_odometry(self) -> None:
        """Publish odometry (pose + velocity)."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = f'{self.robot_id}/base_link'

        # Position
        odom.pose.pose.position.x = self.robot.x
        odom.pose.pose.position.y = self.robot.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from heading)
        quat = tf_transformations.quaternion_from_euler(
            0, 0, self.robot.heading
        ) if HAS_ROS2 else [0, 0, math.sin(self.robot.heading/2), math.cos(self.robot.heading/2)]

        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Velocity (linear and angular)
        odom.twist.twist.linear.x = self.robot.linear_velocity
        odom.twist.twist.angular.z = self.robot.angular_velocity

        self.odom_pub.publish(odom)

    def _publish_battery(self) -> None:
        """Publish battery state."""
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.header.frame_id = f'{self.robot_id}/battery'

        # Convert percent to fraction
        battery_msg.percentage = self.robot.battery.current_charge / 100.0
        battery_msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if self.robot.state == AMRState.CHARGING
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        battery_msg.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_COLD
            if self.robot.battery.is_critical()
            else (BatteryState.POWER_SUPPLY_HEALTH_DEAD
                  if self.robot.battery.current_charge == 0
                  else BatteryState.POWER_SUPPLY_HEALTH_GOOD)
        )

        # Voltage and current (estimated)
        battery_msg.voltage = self.robot.battery.voltage
        battery_msg.current = (
            -0.1 if self.robot.state == AMRState.CHARGING
            else 0.1 if abs(self.robot.linear_velocity) > 0.01
            else 0.01
        )
        battery_msg.charge = (
            self.robot.battery.current_charge / 100.0 * self.robot.battery.capacity_ah
        )
        battery_msg.capacity = self.robot.battery.capacity_ah

        self.battery_pub.publish(battery_msg)

    def get_robot_state_dict(self) -> dict:
        """Return robot state as dictionary for diagnostics."""
        return self.robot.get_status_dict()


def main(args=None):
    """Standalone node entry point."""
    if not HAS_ROS2:
        print("ERROR: ROS2 not installed")
        return

    rclpy.init(args=args)

    # Example: create node for amr_001
    node = AMRDriverNode(
        robot_id='amr_001',
        start_x=0.0,
        start_y=0.0,
        start_heading=0.0,
        update_rate=10
    )

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
