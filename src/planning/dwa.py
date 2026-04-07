"""
Dynamic Window Approach (DWA) obstacle avoidance and trajectory planning.

Implements the DWA algorithm for local reactive obstacle avoidance by evaluating
candidate trajectories and scoring them based on heading error, obstacle distance,
and velocity magnitude.

Mathematical formulas:
  Trajectory simulation: x(t) = x + v*cos(θ)*t, y(t) = y + v*sin(θ)*t, θ(t) = θ + ω*t
  Overall score: G(v,ω) = α*heading + β*distance_to_obstacle + γ*velocity
  Emergency threshold: if min_obstacle_distance < emergency_distance, apply emergency stop

Author: Muskaan Maheshwari
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class DWAMetrics:
    """Metrics from DWA planning execution."""
    best_score: float
    min_obstacle_distance: float
    heading_error: float
    linear_velocity: float
    angular_velocity: float


class DWAPlanner:
    """
    Dynamic Window Approach planner for local obstacle avoidance.

    Evaluates candidate (v, ω) pairs over a prediction window and selects the trajectory
    with the highest composite score considering obstacle proximity, heading alignment,
    and velocity efficiency.

    Attributes:
        max_linear_velocity: Maximum linear velocity in m/s.
        max_angular_velocity: Maximum angular velocity in rad/s.
        prediction_time: Time horizon for trajectory simulation in seconds.
        dt: Time step for trajectory discretization in seconds.
        heading_weight: Weight for heading error component in scoring.
        obstacle_distance_weight: Weight for obstacle distance component.
        velocity_weight: Weight for velocity magnitude component.
        emergency_distance: Minimum safe distance threshold in meters.
        velocity_resolution: Resolution of velocity sampling (m/s increments).
        angular_velocity_resolution: Resolution of angular velocity sampling (rad/s).
    """

    def __init__(
        self,
        max_linear_velocity: float = 1.5,
        max_angular_velocity: float = 2.0,
        prediction_time: float = 2.0,
        dt: float = 0.1,
        heading_weight: float = 0.3,
        obstacle_distance_weight: float = 0.5,
        velocity_weight: float = 0.2,
        emergency_distance: float = 0.3,
        velocity_resolution: float = 0.1,
        angular_velocity_resolution: float = 0.1,
    ) -> None:
        """
        Initialize the DWA planner.

        Args:
            max_linear_velocity: Maximum linear velocity in m/s (default 1.5).
            max_angular_velocity: Maximum angular velocity in rad/s (default 2.0).
            prediction_time: Time horizon for trajectory prediction in seconds (default 2.0).
            dt: Time step for simulation in seconds (default 0.1).
            heading_weight: Weight for heading error in scoring (default 0.3).
            obstacle_distance_weight: Weight for obstacle distance in scoring (default 0.5).
            velocity_weight: Weight for velocity magnitude in scoring (default 0.2).
            emergency_distance: Minimum safe distance threshold in meters (default 0.3).
            velocity_resolution: Resolution of linear velocity sampling in m/s (default 0.1).
            angular_velocity_resolution: Resolution of angular velocity sampling in rad/s (default 0.1).
        """
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.prediction_time = prediction_time
        self.dt = dt
        self.heading_weight = heading_weight
        self.obstacle_distance_weight = obstacle_distance_weight
        self.velocity_weight = velocity_weight
        self.emergency_distance = emergency_distance
        self.velocity_resolution = velocity_resolution
        self.angular_velocity_resolution = angular_velocity_resolution

    def plan(
        self,
        robot_pose: Tuple[float, float, float],
        current_velocity: Tuple[float, float],
        goal: Tuple[float, float],
        obstacle_grid: Optional[np.ndarray] = None,
    ) -> Tuple[Tuple[float, float], DWAMetrics]:
        """
        Plan next velocity command using Dynamic Window Approach.

        Evaluates candidate velocities (v, ω) by simulating trajectories and scoring
        based on heading error, obstacle proximity, and velocity efficiency.

        Args:
            robot_pose: Current robot pose as (x, y, heading_rad).
            current_velocity: Current velocity as (linear_m_s, angular_rad_s).
            goal: Goal position as (goal_x, goal_y).
            obstacle_grid: Optional occupancy grid (m x n numpy array) where 0=free, 1=occupied.
                          Grid assumed to be 0.1m resolution with origin at (0, 0).

        Returns:
            Tuple of (velocity_command, metrics) where:
              - velocity_command: (linear_velocity, angular_velocity)
              - metrics: DWAMetrics with scoring information

        Example:
            planner = DWAPlanner()
            cmd, metrics = planner.plan(
                robot_pose=(5.0, 5.0, 0.0),
                current_velocity=(0.5, 0.0),
                goal=(10.0, 5.0),
                obstacle_grid=grid
            )
            print(f"Best score: {metrics.best_score}")
        """
        x, y, heading = robot_pose
        current_v, current_w = current_velocity

        # Generate candidate velocity pairs
        candidate_velocities = self._generate_candidates(current_v, current_w)

        if not candidate_velocities:
            # No valid candidates, stop
            return (0.0, 0.0), DWAMetrics(
                best_score=0.0,
                min_obstacle_distance=0.0,
                heading_error=0.0,
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # Score each candidate trajectory
        best_score = -float("inf")
        best_velocity = (0.0, 0.0)
        best_metrics = None

        for v, w in candidate_velocities:
            # Simulate trajectory
            trajectory = self._simulate_trajectory(
                x, y, heading, v, w
            )

            # Check obstacles
            min_distance = self._compute_min_obstacle_distance(
                trajectory, obstacle_grid
            )

            # Check for emergency condition
            if min_distance < self.emergency_distance and min_distance > 0:
                # Too close to obstacle
                continue

            # Score trajectory
            heading_score = self._compute_heading_score(trajectory, goal)
            distance_score = self._compute_distance_score(min_distance)
            velocity_score = self._compute_velocity_score(v)

            # Weighted composite score
            score = (
                self.heading_weight * heading_score +
                self.obstacle_distance_weight * distance_score +
                self.velocity_weight * velocity_score
            )

            if score > best_score:
                best_score = score
                best_velocity = (v, w)
                best_metrics = {
                    "min_distance": min_distance,
                    "heading_score": heading_score,
                    "distance_score": distance_score,
                    "velocity_score": velocity_score,
                }

        # If no valid trajectory found, return zero velocity
        if best_metrics is None:
            return (0.0, 0.0), DWAMetrics(
                best_score=best_score,
                min_obstacle_distance=0.0,
                heading_error=0.0,
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # Compute heading error for final trajectory
        final_trajectory = self._simulate_trajectory(
            x, y, heading, best_velocity[0], best_velocity[1]
        )
        heading_error = self._compute_heading_error(final_trajectory, goal)

        metrics = DWAMetrics(
            best_score=best_score,
            min_obstacle_distance=best_metrics["min_distance"],
            heading_error=heading_error,
            linear_velocity=best_velocity[0],
            angular_velocity=best_velocity[1],
        )

        return best_velocity, metrics

    def _generate_candidates(
        self,
        current_v: float,
        current_w: float,
    ) -> List[Tuple[float, float]]:
        """
        Generate candidate velocity pairs within the dynamic window.

        Dynamic window constrains velocity choices based on current acceleration limits.

        Args:
            current_v: Current linear velocity in m/s.
            current_w: Current angular velocity in rad/s.

        Returns:
            List of (v, w) candidate pairs.
        """
        candidates = []

        # Linear velocity range (can accelerate/decelerate within one time step)
        v_min = max(-self.max_linear_velocity,
                    current_v - self.max_linear_velocity * self.dt)
        v_max = min(self.max_linear_velocity,
                    current_v + self.max_linear_velocity * self.dt)

        # Angular velocity range
        w_min = max(-self.max_angular_velocity,
                    current_w - self.max_angular_velocity * self.dt)
        w_max = min(self.max_angular_velocity,
                    current_w + self.max_angular_velocity * self.dt)

        # Sample velocities
        v = v_min
        while v <= v_max:
            w = w_min
            while w <= w_max:
                candidates.append((v, w))
                w += self.angular_velocity_resolution
            v += self.velocity_resolution

        return candidates

    def _simulate_trajectory(
        self,
        x: float,
        y: float,
        heading: float,
        v: float,
        w: float,
    ) -> List[Tuple[float, float, float]]:
        """
        Simulate a trajectory for the given velocity pair.

        Uses simple kinematic model: x(t) = x + v*cos(θ)*t, y(t) = y + v*sin(θ)*t, θ(t) = θ + ω*t

        Args:
            x: Initial x position in meters.
            y: Initial y position in meters.
            heading: Initial heading in radians.
            v: Linear velocity in m/s.
            w: Angular velocity in rad/s.

        Returns:
            List of (x, y, heading) poses sampled along the trajectory.
        """
        trajectory = [(x, y, heading)]
        current_x, current_y, current_h = x, y, heading

        t = self.dt
        while t <= self.prediction_time:
            # Kinematic model
            current_x += v * math.cos(current_h) * self.dt
            current_y += v * math.sin(current_h) * self.dt
            current_h += w * self.dt
            current_h = self._normalize_angle(current_h)

            trajectory.append((current_x, current_y, current_h))
            t += self.dt

        return trajectory

    def _compute_min_obstacle_distance(
        self,
        trajectory: List[Tuple[float, float, float]],
        obstacle_grid: Optional[np.ndarray],
    ) -> float:
        """
        Compute minimum distance to any obstacle along trajectory.

        Checks trajectory against obstacle grid. Assumes grid resolution of 0.1m
        with origin at (0, 0).

        Args:
            trajectory: List of (x, y, heading) poses.
            obstacle_grid: Occupancy grid (0=free, 1=occupied) or None.

        Returns:
            Minimum distance in meters, or float('inf') if no obstacles.
        """
        if obstacle_grid is None:
            return float("inf")

        min_distance = float("inf")
        grid_resolution = 0.1

        for x, y, _ in trajectory:
            # Convert world coordinates to grid indices
            grid_x = int(round(x / grid_resolution))
            grid_y = int(round(y / grid_resolution))

            # Check if within grid bounds
            if 0 <= grid_x < obstacle_grid.shape[1] and 0 <= grid_y < obstacle_grid.shape[0]:
                if obstacle_grid[grid_y, grid_x] > 0:
                    # Found obstacle, compute distance
                    distance = math.sqrt(x**2 + y**2)
                    min_distance = min(min_distance, distance)

        return min_distance

    def _compute_heading_score(
        self,
        trajectory: List[Tuple[float, float, float]],
        goal: Tuple[float, float],
    ) -> float:
        """
        Score trajectory based on alignment toward goal.

        Uses angle to goal at final trajectory point. Returns value in [0, 1].

        Args:
            trajectory: List of (x, y, heading) poses.
            goal: Goal position as (goal_x, goal_y).

        Returns:
            Score in [0, 1], higher is better (0 = pointing away, 1 = pointing at goal).
        """
        if not trajectory:
            return 0.0

        final_x, final_y, final_h = trajectory[-1]
        goal_x, goal_y = goal

        # Angle to goal
        angle_to_goal = math.atan2(goal_y - final_y, goal_x - final_x)

        # Error between heading and goal angle
        error = self._angle_difference(angle_to_goal, final_h)
        error = abs(error)

        # Score: 1 when error=0, 0 when error=π
        score = max(0.0, 1.0 - error / math.pi)

        return score

    def _compute_distance_score(self, min_distance: float) -> float:
        """
        Score trajectory based on distance to nearest obstacle.

        Monotonically increasing with distance. Returns value in [0, 1].

        Args:
            min_distance: Minimum distance to obstacle in meters.

        Returns:
            Score in [0, 1], higher for larger distances.
        """
        # Normalize to reasonable range (0-5m)
        max_safe_distance = 5.0
        normalized = min(min_distance / max_safe_distance, 1.0)

        return normalized

    def _compute_velocity_score(self, v: float) -> float:
        """
        Score trajectory based on velocity magnitude.

        Prefers higher velocities (more efficient) but bounded by maximum.

        Args:
            v: Linear velocity in m/s.

        Returns:
            Score in [0, 1], higher for higher velocities.
        """
        normalized = abs(v) / self.max_linear_velocity
        return min(normalized, 1.0)

    def _compute_heading_error(
        self,
        trajectory: List[Tuple[float, float, float]],
        goal: Tuple[float, float],
    ) -> float:
        """
        Compute heading error toward goal at final trajectory point.

        Args:
            trajectory: List of (x, y, heading) poses.
            goal: Goal position as (goal_x, goal_y).

        Returns:
            Heading error in radians.
        """
        if not trajectory:
            return 0.0

        final_x, final_y, final_h = trajectory[-1]
        goal_x, goal_y = goal

        angle_to_goal = math.atan2(goal_y - final_y, goal_x - final_x)
        error = self._angle_difference(angle_to_goal, final_h)

        return abs(error)

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
        Compute shortest signed angular difference from current to target.

        Args:
            target: Target angle in radians.
            current: Current angle in radians.

        Returns:
            Signed angular difference in radians.
        """
        diff = target - current
        diff = DWAPlanner._normalize_angle(diff)
        return diff
