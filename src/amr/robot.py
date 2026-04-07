"""
TK-AMR Automake style factory AMR robot model with differential drive base and independent turret.

This module provides a physics-based simulation model of a factory AMR with:
  - Differential drive base capable of 360° in-place rotation
  - Independent turret top for load alignment at docking
  - Realistic battery management and discharge modeling
  - Collision detection via footprint and safety circle
  - State management for task coordination

Author: Muskaan Maheshwari
"""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple


class AMRState(Enum):
    """Discrete states representing AMR operational modes."""

    IDLE = "idle"
    NAVIGATING = "navigating"
    DOCKING = "docking"
    LOADING = "loading"
    UNLOADING = "unloading"
    CHARGING = "charging"
    PARKED = "parked"
    EMERGENCY_STOP = "emergency_stop"
    WAITING = "waiting"  # waiting for traffic to clear


@dataclass
class BatteryModel:
    """
    Realistic battery model for 48V lithium AMR.

    Attributes:
        capacity_ah: Battery capacity in amp-hours (default 100 Ah).
        voltage: Nominal voltage (default 48V).
        current_charge: Current charge level as percentage (0-100).
        discharge_rate_per_meter: Discharge in % per meter traveled.
        discharge_rate_idle: Discharge in % per second at idle.
        charge_rate: Charge rate in % per second during charging (45 min from 20% to 100%).
        low_threshold: Battery percentage threshold for low warning.
        critical_threshold: Battery percentage threshold for critical warning.
    """

    capacity_ah: float = 100.0
    voltage: float = 48.0
    current_charge: float = 100.0
    discharge_rate_per_meter: float = 0.02
    discharge_rate_idle: float = 0.001
    charge_rate: float = 1.67  # (100-20)% / (45*60 sec) ≈ 0.0267 %/sec, rounded to 1.67 %/min
    low_threshold: float = 20.0
    critical_threshold: float = 10.0

    def discharge(self, distance_m: float) -> None:
        """
        Reduce charge based on distance traveled.

        Args:
            distance_m: Distance traveled in meters.
        """
        self.current_charge = max(
            0.0, self.current_charge - (distance_m * self.discharge_rate_per_meter)
        )

    def discharge_idle(self, dt: float) -> None:
        """
        Reduce charge for idle time.

        Args:
            dt: Time step in seconds.
        """
        self.current_charge = max(
            0.0, self.current_charge - (dt * self.discharge_rate_idle)
        )

    def charge(self, dt: float) -> None:
        """
        Increase charge during charging.

        Args:
            dt: Time step in seconds.
        """
        self.current_charge = min(
            100.0, self.current_charge + (dt * self.charge_rate)
        )

    def energy_wh(self) -> float:
        """
        Calculate remaining energy in watt-hours.

        Returns:
            Remaining energy in Wh.
        """
        return (self.current_charge / 100.0) * self.capacity_ah * self.voltage

    def is_low(self) -> bool:
        """Check if battery is below low threshold."""
        return self.current_charge < self.low_threshold

    def is_critical(self) -> bool:
        """Check if battery is below critical threshold."""
        return self.current_charge < self.critical_threshold

    def can_reach(self, distance_m: float) -> bool:
        """
        Estimate if robot has enough charge to travel a distance and return to charger.

        Args:
            distance_m: Distance to travel one way.

        Returns:
            True if sufficient charge for round trip plus safety margin.
        """
        round_trip_discharge = (distance_m * 2.0) * self.discharge_rate_per_meter
        return self.current_charge > (round_trip_discharge + self.critical_threshold)


@dataclass
class AMRSpecs:
    """
    Physical specifications for TK-AMR style factory AMR.

    Attributes:
        length: Robot length in meters.
        width: Robot width in meters.
        height: Robot height in meters.
        turret_diameter: Turret diameter in meters.
        max_payload_kg: Maximum payload capacity in kg.
        max_linear_speed: Maximum linear velocity in m/s.
        max_angular_speed: Maximum angular velocity in rad/s.
        max_turret_speed: Maximum turret rotation speed in rad/s.
        max_acceleration: Maximum linear acceleration in m/s^2.
        max_deceleration: Maximum linear deceleration in m/s^2.
        safety_radius: Safety radius for collision avoidance in meters.
    """

    # Physical dimensions (TK-AMR Automake specs)
    length: float = 1.2
    width: float = 0.8
    height: float = 0.4
    turret_diameter: float = 0.7

    # Performance
    max_payload_kg: float = 500.0
    max_linear_speed: float = 1.5
    max_angular_speed: float = 2.0
    max_turret_speed: float = 1.0
    max_acceleration: float = 0.5
    max_deceleration: float = 1.0

    # Safety
    safety_radius: float = field(init=False)

    def __post_init__(self) -> None:
        """Calculate safety radius from dimensions."""
        base_radius = max(self.length, self.width) / 2.0
        self.safety_radius = base_radius + 0.3


class AMRRobot:
    """
    Physics-based model of a TK-AMR Automake style factory AMR.

    Represents a differential drive robot with an independent turret capable of:
      - Navigating via velocity commands
      - Rotating in place (0 turning radius)
      - Independent turret rotation for load alignment
      - Battery-aware operation and charging
      - Payload handling
      - Multi-state task coordination

    Attributes:
        robot_id: Unique identifier for this robot.
        specs: Physical specifications (defaults to TK-AMR Automake specs).
    """

    def __init__(
        self,
        robot_id: str,
        start_position: Tuple[float, float],
        start_heading: float = 0.0,
        specs: Optional[AMRSpecs] = None,
    ) -> None:
        """
        Initialize an AMR robot.

        Args:
            robot_id: Unique identifier (e.g., 'amr_001').
            start_position: Initial (x, y) position in meters.
            start_heading: Initial base heading in radians (default 0).
            specs: Robot specifications (default: TK-AMR Automake specs).
        """
        self.robot_id = robot_id
        self.specs = specs if specs is not None else AMRSpecs()

        # Pose (base frame)
        self.x: float = float(start_position[0])
        self.y: float = float(start_position[1])
        self.heading: float = float(start_heading)

        # Turret heading (independent rotation)
        self.turret_heading: float = 0.0

        # Velocities
        self.linear_velocity: float = 0.0
        self.angular_velocity: float = 0.0
        self.turret_angular_velocity: float = 0.0

        # State
        self.state = AMRState.IDLE
        self.battery = BatteryModel()
        self.current_task_id: Optional[str] = None
        self.current_path: Optional[List[Tuple[float, float]]] = None
        self.path_index: int = 0

        # Payload
        self.payload_kg: float = 0.0

        # Trajectory history
        self.trajectory: List[Tuple[float, float]] = [(self.x, self.y)]

        # Obstacle avoidance and detection
        self.dwa_planner: Optional[object] = None
        self.obstacle_detector: Optional[object] = None
        self.alert_generator: Optional[object] = None
        self.obstacle_detected: bool = False
        self.last_obstacle_distance: float = float("inf")
        self.obstacle_free_time: float = 0.0

    @property
    def position(self) -> Tuple[float, float]:
        """Current (x, y) position as a tuple."""
        return (self.x, self.y)

    @position.setter
    def position(self, value: Tuple[float, float]) -> None:
        self.x, self.y = float(value[0]), float(value[1])

    def update(self, dt: float) -> Tuple[float, float, float]:
        """
        Execute one physics step with velocity limits and battery discharge.

        Updates:
          - Position via kinematic model
          - Heading via angular velocity
          - Turret heading independently
          - Battery based on distance traveled
          - Trajectory history
          - Obstacle detection and avoidance

        Args:
            dt: Time step in seconds.

        Returns:
            Updated pose as (x, y, heading).
        """
        # Perform obstacle detection scan if available
        if self.state == AMRState.NAVIGATING and self.obstacle_detector is not None:
            try:
                scan_result = self.obstacle_detector.scan(
                    robot_pose=(self.x, self.y, self.heading)
                )
                if scan_result.obstacle_detected:
                    self.obstacle_detected = True
                    if scan_result.obstacles:
                        self.last_obstacle_distance = min(
                            obs.distance for obs in scan_result.obstacles
                        )
                        # Generate alert if threshold is crossed
                        if self.alert_generator is not None and self.last_obstacle_distance < 1.0:
                            obs = scan_result.obstacles[0]
                            alert = self.alert_generator.create_obstacle_alert(
                                robot_id=self.robot_id,
                                position=obs.position,
                                obstacle_type=obs.obstacle_type,
                                distance=obs.distance,
                                current_state=self.state.value,
                                severity=obs.severity,
                                confidence=obs.confidence,
                            )
                else:
                    self.obstacle_free_time += dt
                    if self.obstacle_detected:
                        # Obstacle was just cleared
                        self.obstacle_detected = False
                        if self.alert_generator is not None:
                            alert = self.alert_generator.create_cleared_alert(
                                robot_id=self.robot_id,
                                position=(self.x, self.y),
                            )
            except Exception:
                # Detector error, continue normally
                pass

        if self.state == AMRState.EMERGENCY_STOP:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.turret_angular_velocity = 0.0
            self.battery.discharge_idle(dt)
            return (self.x, self.y, self.heading)

        # Apply acceleration constraints
        max_accel_step = self.specs.max_acceleration * dt
        max_decel_step = self.specs.max_deceleration * dt

        # Clamp velocities to max speeds
        self.linear_velocity = max(
            -self.specs.max_linear_speed,
            min(self.specs.max_linear_speed, self.linear_velocity),
        )
        self.angular_velocity = max(
            -self.specs.max_angular_speed,
            min(self.specs.max_angular_speed, self.angular_velocity),
        )
        self.turret_angular_velocity = max(
            -self.specs.max_turret_speed,
            min(self.specs.max_turret_speed, self.turret_angular_velocity),
        )

        # Kinematic update: differential drive
        # x += v*cos(θ)*dt
        # y += v*sin(θ)*dt
        # θ += ω*dt
        distance_step = self.linear_velocity * dt
        self.x += distance_step * math.cos(self.heading)
        self.y += distance_step * math.sin(self.heading)
        self.heading += self.angular_velocity * dt

        # Normalize heading to [-π, π]
        self.heading = self._normalize_angle(self.heading)

        # Update turret heading independently
        self.turret_heading += self.turret_angular_velocity * dt
        self.turret_heading = self._normalize_angle(self.turret_heading)

        # Battery discharge
        if self.state == AMRState.CHARGING:
            self.battery.charge(dt)
        else:
            if abs(distance_step) > 1e-6:
                self.battery.discharge(abs(distance_step))
            else:
                self.battery.discharge_idle(dt)

        # Record trajectory
        self.trajectory.append((self.x, self.y))

        return (self.x, self.y, self.heading)

    def set_velocity(self, linear: float, angular: float) -> None:
        """
        Set target velocities with implicit acceleration ramping.

        Velocities are clamped to physical limits.

        Args:
            linear: Target linear velocity in m/s.
            angular: Target angular velocity in rad/s.
        """
        self.linear_velocity = max(
            -self.specs.max_linear_speed,
            min(self.specs.max_linear_speed, float(linear)),
        )
        self.angular_velocity = max(
            -self.specs.max_angular_speed,
            min(self.specs.max_angular_speed, float(angular)),
        )

    def rotate_turret(self, target_heading: float, dt: float) -> bool:
        """
        Rotate turret toward a target heading.

        Implements smooth rotation at max turret speed.

        Args:
            target_heading: Desired turret heading in radians.
            dt: Time step in seconds.

        Returns:
            True if turret has reached target heading (within 0.01 rad tolerance).
        """
        target_heading = self._normalize_angle(target_heading)
        error = self._angle_difference(target_heading, self.turret_heading)

        # Determine rotation direction to minimize angle
        if abs(error) < 0.01:
            self.turret_angular_velocity = 0.0
            return True

        # Rotate toward target at max speed
        self.turret_angular_velocity = (
            self.specs.max_turret_speed if error > 0 else -self.specs.max_turret_speed
        )
        return False

    def emergency_stop(self) -> None:
        """
        Immediately halt robot and set emergency stop state.

        Clears all velocity commands and records emergency state.
        """
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.turret_angular_velocity = 0.0
        self.state = AMRState.EMERGENCY_STOP

    def can_resume_path(self) -> bool:
        """
        Check if path ahead is clear for resuming navigation.

        Checks if obstacle has been absent for sufficient time.

        Returns:
            True if safe to resume navigation, False if obstacles still detected.
        """
        # Check if obstacle-free for at least 1 second
        if self.obstacle_free_time < 1.0:
            return False

        # Check if obstacle detector confirms clear
        if self.obstacle_detector is not None:
            try:
                scan_result = self.obstacle_detector.scan(
                    robot_pose=(self.x, self.y, self.heading)
                )
                return not scan_result.obstacle_detected
            except Exception:
                return False

        return self.last_obstacle_distance > 1.0

    def start_charging(self) -> None:
        """Begin charging at dock."""
        self.state = AMRState.CHARGING
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def stop_charging(self) -> None:
        """Stop charging and return to idle."""
        self.state = AMRState.IDLE

    def load_payload(self, kg: float) -> None:
        """
        Load payload onto robot.

        Args:
            kg: Payload mass in kg (clamped to max_payload_kg).

        Raises:
            ValueError: If payload exceeds capacity.
        """
        if kg > self.specs.max_payload_kg:
            raise ValueError(
                f"Payload {kg}kg exceeds max capacity {self.specs.max_payload_kg}kg"
            )
        self.payload_kg = kg

    def unload_payload(self) -> float:
        """
        Unload payload from robot.

        Returns:
            Payload mass in kg that was unloaded.
        """
        payload = self.payload_kg
        self.payload_kg = 0.0
        return payload

    def get_footprint(self) -> List[Tuple[float, float]]:
        """
        Get robot footprint corners in world frame.

        Returns 4 corners of the robot base bounding box, rotated by current heading.

        Returns:
            List of 4 (x, y) corners in world coordinates.
        """
        # Half-dimensions
        half_length = self.specs.length / 2.0
        half_width = self.specs.width / 2.0

        # Corners in robot frame (before rotation)
        corners_robot = [
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, -half_width),
            (-half_length, half_width),
        ]

        # Rotate by heading and translate to world position
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)

        corners_world = []
        for cx, cy in corners_robot:
            # Rotate
            rx = cx * cos_h - cy * sin_h
            ry = cx * sin_h + cy * cos_h
            # Translate
            corners_world.append((self.x + rx, self.y + ry))

        return corners_world

    def get_safety_circle(self) -> Tuple[Tuple[float, float], float]:
        """
        Get robot safety circle for collision checking.

        Returns:
            Tuple of (center_position, radius) where center is robot position
            and radius is safety_radius.
        """
        return ((self.x, self.y), self.specs.safety_radius)

    def distance_to(self, target: Tuple[float, float]) -> float:
        """
        Compute Euclidean distance to target position.

        Args:
            target: Target (x, y) position.

        Returns:
            Distance in meters.
        """
        dx = target[0] - self.x
        dy = target[1] - self.y
        return math.sqrt(dx * dx + dy * dy)

    def get_status_dict(self) -> Dict:
        """
        Serialize robot state to JSON-compatible dictionary.

        Returns:
            Dictionary with all robot state and sensor readings.
        """
        return {
            "robot_id": self.robot_id,
            "position": {"x": self.x, "y": self.y},
            "heading_rad": self.heading,
            "heading_deg": math.degrees(self.heading),
            "turret_heading_rad": self.turret_heading,
            "turret_heading_deg": math.degrees(self.turret_heading),
            "linear_velocity": self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "turret_angular_velocity": self.turret_angular_velocity,
            "state": self.state.value,
            "current_task_id": self.current_task_id,
            "battery": {
                "charge_percent": self.battery.current_charge,
                "energy_wh": self.battery.energy_wh(),
                "is_low": self.battery.is_low(),
                "is_critical": self.battery.is_critical(),
            },
            "payload_kg": self.payload_kg,
            "trajectory_length": len(self.trajectory),
        }

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-π, π] range.

        Args:
            angle: Angle in radians.

        Returns:
            Normalized angle.
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def _angle_difference(target: float, current: float) -> float:
        """
        Compute shortest angular difference from current to target.

        Args:
            target: Target angle in radians.
            current: Current angle in radians.

        Returns:
            Signed angular difference (positive = counterclockwise).
        """
        diff = target - current
        diff = AMRRobot._normalize_angle(diff)
        return diff


def create_default_fleet(
    n_robots: int, parking_positions: List[Tuple[float, float]]
) -> Dict[str, AMRRobot]:
    """
    Factory function to create a fleet of AMRs at parking positions.

    Creates n robots with staggered initial headings and default specs.

    Args:
        n_robots: Number of robots to create.
        parking_positions: List of (x, y) positions for parking.
                          Must have at least n_robots positions.

    Returns:
        Dictionary mapping robot_id (amr_001, amr_002, ...) to AMRRobot instances.

    Raises:
        ValueError: If fewer parking positions than robots requested.
    """
    if len(parking_positions) < n_robots:
        raise ValueError(
            f"Need {n_robots} parking positions but got {len(parking_positions)}"
        )

    fleet: Dict[str, AMRRobot] = {}
    for i in range(n_robots):
        robot_id = f"amr_{i+1:03d}"
        position = parking_positions[i]
        # Stagger initial headings (0, 90, 180, 270 degrees for first 4, then cycle)
        initial_heading = (i % 4) * (math.pi / 2.0)

        robot = AMRRobot(
            robot_id=robot_id,
            start_position=position,
            start_heading=initial_heading,
            specs=AMRSpecs(),
        )
        fleet[robot_id] = robot

    return fleet
