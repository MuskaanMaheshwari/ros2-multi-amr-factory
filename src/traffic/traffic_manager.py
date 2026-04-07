"""Traffic Manager for multi-robot coordination in factory environments.

Handles intersection control, priority-based yielding, and deadlock detection/resolution.

Author: Muskaan Maheshwari
"""

import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Tuple

import numpy as np


class IntersectionState(IntEnum):
    """State of an intersection in the traffic network."""

    FREE = 0  # No robot claiming this intersection
    CLAIMED = 1  # A robot has reserved it
    OCCUPIED = 2  # A robot is physically inside it


class TrafficPriority(IntEnum):
    """Priority levels for robot tasks."""

    EMERGENCY = 0  # Emergency vehicles / low battery returning to charge
    HIGH = 1  # Urgent production tasks
    NORMAL = 2  # Standard delivery tasks
    LOW = 3  # Parking / idle repositioning


@dataclass
class Intersection:
    """Represents a traffic intersection in the factory.

    Attributes:
        intersection_id: Unique identifier for this intersection
        center: (x, y) coordinates of intersection center
        radius: Zone of control in meters (default 3.0)
        state: Current state (FREE, CLAIMED, OCCUPIED)
        claimed_by: Robot ID that claimed this intersection, if any
        queue: List of robot IDs waiting for this intersection (sorted by priority)
        last_clear_time: Timestamp when intersection last became FREE
    """

    intersection_id: str
    center: Tuple[float, float]
    radius: float = 3.0
    state: IntersectionState = IntersectionState.FREE
    claimed_by: Optional[str] = None
    queue: List[str] = field(default_factory=list)
    last_clear_time: float = field(default_factory=time.time)


@dataclass
class DeadlockInfo:
    """Information about a detected deadlock condition.

    Attributes:
        robot_ids: List of robot IDs involved in the deadlock
        intersection_ids: List of intersection IDs involved
        detected_time: Timestamp when deadlock was detected
        resolved: Whether the deadlock has been resolved
        resolution_method: Method used to resolve ('reroute', 'yield', 'backup')
    """

    robot_ids: List[str]
    intersection_ids: List[str]
    detected_time: float
    resolved: bool = False
    resolution_method: str = ""


class TrafficManager:
    """Manages traffic coordination for multiple robots in a factory.

    Prevents collisions at intersections, manages right-of-way, and detects/resolves deadlocks.

    Attributes:
        intersections: Dict mapping intersection_id to Intersection objects
        safety_radius: Minimum safe distance between any two robots (meters)
        robot_priorities: Dict mapping robot_id to their current TrafficPriority
        robot_battery: Dict mapping robot_id to battery level (0.0-1.0)
        active_deadlocks: Dict mapping deadlock identifier to DeadlockInfo
    """

    def __init__(
        self,
        intersections: List[Tuple[float, float]],
        safety_radius: float = 1.5,
    ) -> None:
        """Initialize the traffic manager with intersection locations.

        Args:
            intersections: List of (x, y) coordinates for intersection centers
            safety_radius: Minimum safe distance between robots in meters
        """
        self.intersections: Dict[str, Intersection] = {}
        self.safety_radius = safety_radius
        self.robot_priorities: Dict[str, TrafficPriority] = {}
        self.robot_battery: Dict[str, float] = {}
        self.active_deadlocks: Dict[str, DeadlockInfo] = {}

        # Create Intersection objects from position list
        for i, (x, y) in enumerate(intersections):
            intersection_id = f"intersection_{i}"
            self.intersections[intersection_id] = Intersection(
                intersection_id=intersection_id,
                center=(float(x), float(y)),
            )

    def set_robot_priority(self, robot_id: str, priority: TrafficPriority) -> None:
        """Set the priority level for a robot.

        Args:
            robot_id: Identifier of the robot
            priority: Priority level for this robot
        """
        self.robot_priorities[robot_id] = priority

    def set_robot_battery(self, robot_id: str, battery_level: float) -> None:
        """Set the battery level for a robot.

        Args:
            robot_id: Identifier of the robot
            battery_level: Battery level as fraction 0.0-1.0
        """
        self.robot_battery[robot_id] = max(0.0, min(1.0, battery_level))

    def request_intersection(
        self,
        robot_id: str,
        intersection_id: str,
        priority: TrafficPriority,
    ) -> bool:
        """Request access to an intersection.

        If the intersection is FREE, claim it immediately. If CLAIMED or OCCUPIED,
        add the robot to the queue sorted by priority.

        Args:
            robot_id: Identifier of requesting robot
            intersection_id: Identifier of target intersection
            priority: Priority level of this request

        Returns:
            True if intersection is immediately available, False if queued
        """
        if intersection_id not in self.intersections:
            return False

        intersection = self.intersections[intersection_id]

        # Already claimed by this robot
        if intersection.claimed_by == robot_id:
            return True

        # Intersection is free
        if intersection.state == IntersectionState.FREE:
            intersection.state = IntersectionState.CLAIMED
            intersection.claimed_by = robot_id
            self.set_robot_priority(robot_id, priority)
            return True

        # Intersection is claimed or occupied by another robot
        # Add to queue if not already there
        if robot_id not in intersection.queue:
            intersection.queue.append(robot_id)
            self.set_robot_priority(robot_id, priority)
            # Sort queue by priority (lower value = higher priority)
            intersection.queue.sort(
                key=lambda rid: (
                    self.robot_priorities.get(rid, TrafficPriority.NORMAL),
                    rid,  # Tiebreaker: robot ID
                )
            )

        return False

    def enter_intersection(self, robot_id: str, intersection_id: str) -> bool:
        """Mark robot as physically entering an intersection.

        Only succeeds if the robot has claimed this intersection.

        Args:
            robot_id: Identifier of robot entering
            intersection_id: Identifier of intersection

        Returns:
            True if robot successfully entered, False otherwise
        """
        if intersection_id not in self.intersections:
            return False

        intersection = self.intersections[intersection_id]

        # Only allow if this robot claimed it
        if intersection.claimed_by != robot_id:
            return False

        intersection.state = IntersectionState.OCCUPIED
        return True

    def exit_intersection(self, robot_id: str, intersection_id: str) -> None:
        """Mark robot as exiting an intersection.

        Clears the intersection and grants it to the next waiting robot if any.

        Args:
            robot_id: Identifier of robot exiting
            intersection_id: Identifier of intersection
        """
        if intersection_id not in self.intersections:
            return

        intersection = self.intersections[intersection_id]

        # Only clear if this robot currently owns it
        if intersection.claimed_by != robot_id:
            return

        intersection.state = IntersectionState.FREE
        intersection.claimed_by = None
        intersection.last_clear_time = time.time()

        # Grant to next in queue
        if intersection.queue:
            next_robot_id = intersection.queue.pop(0)
            intersection.state = IntersectionState.CLAIMED
            intersection.claimed_by = next_robot_id

    def check_path_intersections(
        self, path: List[Tuple[float, float]]
    ) -> List[str]:
        """Find all intersections that a path passes through.

        A path passes through an intersection if any point is within its radius.

        Args:
            path: List of (x, y) waypoints

        Returns:
            List of intersection IDs that the path intersects
        """
        intersecting_ids = []

        for intersection_id, intersection in self.intersections.items():
            cx, cy = intersection.center
            radius = intersection.radius

            # Check if any point in path is within intersection radius
            for px, py in path:
                dist = np.sqrt((px - cx) ** 2 + (py - cy) ** 2)
                if dist <= radius:
                    intersecting_ids.append(intersection_id)
                    break  # Found this intersection, move to next

        return intersecting_ids

    def check_collision_risk(
        self, robot_positions: Dict[str, Tuple[float, float]]
    ) -> List[Tuple[str, str]]:
        """Detect pairs of robots at collision risk.

        Returns pairs where distance between robots is within safety_radius.

        Args:
            robot_positions: Dict mapping robot_id to (x, y) position

        Returns:
            List of (robot_id_1, robot_id_2) tuples in collision risk
        """
        collision_pairs = []
        robot_ids = list(robot_positions.keys())

        for i, robot_a in enumerate(robot_ids):
            for robot_b in robot_ids[i + 1 :]:
                x_a, y_a = robot_positions[robot_a]
                x_b, y_b = robot_positions[robot_b]

                dist = np.sqrt((x_a - x_b) ** 2 + (y_a - y_b) ** 2)

                if dist < self.safety_radius:
                    collision_pairs.append((robot_a, robot_b))

        return collision_pairs

    def detect_deadlock(
        self, robot_states: Dict[str, Dict]
    ) -> Optional[DeadlockInfo]:
        """Detect deadlock conditions using wait-for graph analysis.

        A deadlock occurs when robots form a cycle where each waits for
        an intersection held by another robot in the cycle.

        Args:
            robot_states: Dict mapping robot_id to state dict with keys:
                - 'position': (x, y) tuple
                - 'waiting_for': intersection_id (optional)
                - 'claiming': intersection_id (optional)

        Returns:
            DeadlockInfo if deadlock detected, None otherwise
        """
        # Build wait-for graph: robot_id -> intersection_id it's waiting for
        wait_graph: Dict[str, Optional[str]] = {}

        for robot_id, state in robot_states.items():
            waiting_for = state.get("waiting_for")
            wait_graph[robot_id] = waiting_for

        # DFS to detect cycles
        visited: Dict[str, int] = {}  # 0: unvisited, 1: visiting, 2: visited
        path: List[str] = []

        def has_cycle(node: str) -> bool:
            """Check if node is part of a cycle using DFS."""
            visited[node] = 1  # Mark as visiting
            path.append(node)

            waiting_for = wait_graph.get(node)
            if waiting_for is None:
                visited[node] = 2
                path.pop()
                return False

            # Find robot holding the intersection
            holder = self.intersections.get(waiting_for, Intersection("", (0, 0))).claimed_by
            if holder is None:
                visited[node] = 2
                path.pop()
                return False

            if visited.get(holder, 0) == 1:
                # Found cycle
                cycle_start_idx = path.index(holder)
                cycle_robots = path[cycle_start_idx:] + [holder]
                cycle_intersections = [
                    wait_graph.get(rid) for rid in cycle_robots[:-1]
                ]
                cycle_intersections = [iid for iid in cycle_intersections if iid]

                deadlock = DeadlockInfo(
                    robot_ids=cycle_robots[:-1],
                    intersection_ids=cycle_intersections,
                    detected_time=time.time(),
                )
                return True

            if visited.get(holder, 0) == 0:
                if has_cycle(holder):
                    return True

            visited[node] = 2
            path.pop()
            return False

        # Initialize visited dict
        for robot_id in wait_graph.keys():
            if visited.get(robot_id, 0) == 0:
                if has_cycle(robot_id):
                    # Extract cycle info
                    cycle_start_idx = len(path) - 2
                    cycle_robots = path[cycle_start_idx:]
                    cycle_intersections = [
                        wait_graph.get(rid) for rid in cycle_robots
                    ]
                    cycle_intersections = [iid for iid in cycle_intersections if iid]

                    return DeadlockInfo(
                        robot_ids=cycle_robots,
                        intersection_ids=cycle_intersections,
                        detected_time=time.time(),
                    )

        return None

    def resolve_deadlock(
        self, deadlock: DeadlockInfo, robot_states: Dict[str, Dict]
    ) -> Dict[str, str]:
        """Resolve a deadlock using a tiered strategy.

        Resolution priority:
        1. Lower priority robot yields
        2. Higher battery robot yields
        3. Higher robot ID yields

        Args:
            deadlock: DeadlockInfo from detect_deadlock
            robot_states: Dict mapping robot_id to state dict with keys:
                - 'battery': battery level (0.0-1.0)

        Returns:
            Dict mapping robot_id to action ('yield', 'proceed', 'reroute')
        """
        actions: Dict[str, str] = {}

        if len(deadlock.robot_ids) < 2:
            return actions

        # Determine yielding robot
        primary_robot = deadlock.robot_ids[0]
        secondary_robot = deadlock.robot_ids[1] if len(deadlock.robot_ids) > 1 else primary_robot

        primary_priority = self.robot_priorities.get(
            primary_robot, TrafficPriority.NORMAL
        )
        secondary_priority = self.robot_priorities.get(
            secondary_robot, TrafficPriority.NORMAL
        )

        # Lower priority yields
        if primary_priority > secondary_priority:
            actions[primary_robot] = "yield"
            actions[secondary_robot] = "proceed"
        elif secondary_priority > primary_priority:
            actions[secondary_robot] = "yield"
            actions[primary_robot] = "proceed"
        else:
            # Equal priority: higher battery proceeds
            primary_battery = robot_states.get(primary_robot, {}).get("battery", 0.5)
            secondary_battery = robot_states.get(secondary_robot, {}).get("battery", 0.5)

            if primary_battery > secondary_battery:
                actions[secondary_robot] = "yield"
                actions[primary_robot] = "proceed"
            elif secondary_battery > primary_battery:
                actions[primary_robot] = "yield"
                actions[secondary_robot] = "proceed"
            else:
                # Tiebreaker: higher robot ID yields
                if primary_robot > secondary_robot:
                    actions[primary_robot] = "yield"
                    actions[secondary_robot] = "proceed"
                else:
                    actions[secondary_robot] = "yield"
                    actions[primary_robot] = "proceed"

        # For additional robots in deadlock
        for robot_id in deadlock.robot_ids[2:]:
            actions[robot_id] = "yield"

        deadlock.resolved = True
        deadlock.resolution_method = "yield"

        return actions

    def get_velocity_modifier(
        self,
        robot_id: str,
        robot_position: Tuple[float, float],
        robot_velocity: Tuple[float, float],
    ) -> float:
        """Calculate velocity scaling factor based on intersection proximity.

        Returns 1.0 for full speed, 0.0 to stop, or intermediate values for
        smooth deceleration as the robot approaches an intersection.

        Args:
            robot_id: Identifier of robot
            robot_position: (x, y) current position
            robot_velocity: (vx, vy) current velocity

        Returns:
            Velocity scaling factor in range [0.0, 1.0]
        """
        min_modifier = 1.0
        rx, ry = robot_position

        for intersection_id, intersection in self.intersections.items():
            cx, cy = intersection.center
            radius = intersection.radius

            # Distance to intersection center
            dist = np.sqrt((rx - cx) ** 2 + (ry - cy) ** 2)

            # Only consider intersections within 2x radius
            influence_radius = 2 * radius
            if dist > influence_radius:
                continue

            # Determine if moving towards intersection
            vx, vy = robot_velocity
            if vx == 0 and vy == 0:
                speed = 0
            else:
                speed = np.sqrt(vx**2 + vy**2)

            if speed > 1e-6:
                # Check if moving towards intersection
                dx, dy = cx - rx, cy - ry
                dot_product = dx * vx + dy * vy
                if dot_product < 0:
                    # Moving away, no slowdown
                    continue

            # Smoothly decelerate: modifier = max(0, (dist - radius) / radius)
            if dist <= radius:
                # Inside intersection, must stop if not claimed
                if intersection.claimed_by != robot_id:
                    min_modifier = 0.0
                else:
                    min_modifier = 1.0
            else:
                # Outside intersection, smooth deceleration
                modifier = (dist - radius) / radius
                min_modifier = min(min_modifier, max(0.0, modifier))

        return min_modifier

    def update(
        self,
        robot_positions: Dict[str, Tuple[float, float]],
        dt: float,
    ) -> None:
        """Update traffic state based on robot positions.

        Auto-releases intersections when robots exit, updates timing stats.

        Args:
            robot_positions: Dict mapping robot_id to (x, y) position
            dt: Time delta since last update (seconds)
        """
        for intersection_id, intersection in self.intersections.items():
            if intersection.claimed_by is None:
                continue

            claimed_robot_id = intersection.claimed_by
            if claimed_robot_id not in robot_positions:
                continue

            robot_pos = robot_positions[claimed_robot_id]
            cx, cy = intersection.center
            dist = np.sqrt((robot_pos[0] - cx) ** 2 + (robot_pos[1] - cy) ** 2)

            # Auto-release if robot has exited
            if dist > intersection.radius * 1.5:
                if intersection.state == IntersectionState.OCCUPIED:
                    self.exit_intersection(claimed_robot_id, intersection_id)

    def get_status(self) -> Dict:
        """Get overall traffic status.

        Returns:
            Dict with keys:
                - 'num_free': Number of free intersections
                - 'num_claimed': Number of claimed intersections
                - 'num_occupied': Number of occupied intersections
                - 'active_deadlocks': Number of active deadlocks
                - 'queue_lengths': Dict mapping intersection_id to queue length
        """
        num_free = sum(
            1 for i in self.intersections.values()
            if i.state == IntersectionState.FREE
        )
        num_claimed = sum(
            1 for i in self.intersections.values()
            if i.state == IntersectionState.CLAIMED
        )
        num_occupied = sum(
            1 for i in self.intersections.values()
            if i.state == IntersectionState.OCCUPIED
        )

        queue_lengths = {
            iid: len(intersection.queue)
            for iid, intersection in self.intersections.items()
        }

        return {
            "num_free": num_free,
            "num_claimed": num_claimed,
            "num_occupied": num_occupied,
            "active_deadlocks": len(self.active_deadlocks),
            "queue_lengths": queue_lengths,
        }
