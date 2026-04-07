"""Unified factory path planner combining A* graph search with Dubins/spline smoothing.

Integrates the aisle waypoint graph with curved motion primitives for realistic
factory navigation.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import math
import time
import heapq
import numpy as np

from .dubins import DubinsPlanner, DubinsPath
from .spline import CubicSplineSmoother, SplinePath


@dataclass
class FactoryEnvironment:
    """Representation of the factory environment with aisle network.

    Expected structure:
    - aisle_waypoints: Dict[str, Tuple[float, float]] — node_id -> (x, y)
    - aisle_connections: Dict[str, List[str]] — node_id -> list of neighboring node_ids
    - stations: Dict[str, Dict] — station_id -> {'approach_point': (x, y), 'approach_heading': float, ...}
    """
    aisle_waypoints: Dict[str, Tuple[float, float]]
    aisle_connections: Dict[str, List[str]]
    stations: Dict[str, Dict] = field(default_factory=dict)


@dataclass
class PlannerStats:
    """Statistics from a planning execution."""
    path_length: float
    computation_time: float
    num_waypoints: int
    max_curvature: float
    path_type: str  # 'astar+spline', 'dubins', or 'dubins+spline'
    num_aisle_nodes: int = 0
    astar_distance: float = 0.0


class FactoryPathPlanner:
    """Plan optimal paths in the factory aisle network with curved segments."""

    def __init__(
        self,
        factory_env: FactoryEnvironment,
        turning_radius: float = 2.0,
    ) -> None:
        """Initialize the factory path planner.

        Args:
            factory_env: FactoryEnvironment with aisle graph and stations.
            turning_radius: Minimum turning radius in meters.
        """
        self.factory_env = factory_env
        self.turning_radius = turning_radius
        self.dubins_planner = DubinsPlanner(turning_radius)
        self.spline_smoother = CubicSplineSmoother(turning_radius)
        self._EPSILON = 1e-6

    def plan(
        self,
        start_pose: Tuple[float, float, float],
        goal_pose: Tuple[float, float, float],
        obstacle_grid: Optional[np.ndarray] = None,
    ) -> Tuple[List[Tuple[float, float, float]], PlannerStats]:
        """Plan a path from start to goal using A* graph search + smoothing.

        Args:
            start_pose: (x, y, theta) start configuration.
            goal_pose: (x, y, theta) goal configuration.
            obstacle_grid: Optional obstacle grid for collision checking.

        Returns:
            Tuple of (waypoints list, PlannerStats dict).
        """
        start_time = time.time()
        start_pos = start_pose[:2]
        goal_pos = goal_pose[:2]

        # Step 1: Find nearest aisle waypoints
        start_waypoint = self._find_nearest_waypoint(start_pos)
        goal_waypoint = self._find_nearest_waypoint(goal_pos)

        if start_waypoint is None or goal_waypoint is None:
            # Fallback to pure Dubins if no aisle waypoints available
            dubins_path = self.dubins_planner.plan(start_pose, goal_pose)
            if dubins_path is None:
                # Return straight line as last resort
                waypoints = [start_pose, goal_pose]
                stats = PlannerStats(
                    path_length=math.sqrt((goal_pos[0] - start_pos[0])**2 +
                                         (goal_pos[1] - start_pos[1])**2),
                    computation_time=time.time() - start_time,
                    num_waypoints=len(waypoints),
                    max_curvature=0.0,
                    path_type="straight_line",
                )
                return waypoints, stats

            waypoints = self.dubins_planner.sample_path(dubins_path, start_pose)
            stats = PlannerStats(
                path_length=dubins_path.total_length,
                computation_time=time.time() - start_time,
                num_waypoints=len(waypoints),
                max_curvature=1.0 / self.turning_radius,
                path_type="dubins",
            )
            return waypoints, stats

        # Step 2: A* search on aisle graph
        aisle_path = self._astar_on_graph(start_waypoint, goal_waypoint)
        aisle_distance = self._compute_path_distance(aisle_path)

        if aisle_path is None or len(aisle_path) == 0:
            # No aisle path found, use Dubins
            dubins_path = self.dubins_planner.plan(start_pose, goal_pose)
            if dubins_path is None:
                waypoints = [start_pose, goal_pose]
            else:
                waypoints = self.dubins_planner.sample_path(dubins_path, start_pose)

            stats = PlannerStats(
                path_length=math.sqrt((goal_pos[0] - start_pos[0])**2 +
                                     (goal_pos[1] - start_pos[1])**2),
                computation_time=time.time() - start_time,
                num_waypoints=len(waypoints),
                max_curvature=1.0 / self.turning_radius,
                path_type="dubins_fallback",
                num_aisle_nodes=len(self.factory_env.aisle_waypoints),
                astar_distance=0.0,
            )
            return waypoints, stats

        # Step 3: Build waypoint sequence from aisle path
        aisle_waypoints_list: List[Tuple[float, float]] = []
        for node_id in aisle_path:
            if node_id in self.factory_env.aisle_waypoints:
                aisle_waypoints_list.append(
                    self.factory_env.aisle_waypoints[node_id]
                )

        # Step 4: Prepend and append start/goal segments
        full_waypoints: List[Tuple[float, float]] = [start_pos]
        full_waypoints.extend(aisle_waypoints_list)
        full_waypoints.append(goal_pos)

        # Step 5: Smooth entire path with cubic splines
        try:
            spline_path = self.spline_smoother.smooth(
                full_waypoints, num_samples=300
            )
            smooth_points = spline_path.smooth_points
            max_curvature = spline_path.max_curvature
            path_length = spline_path.total_length
        except Exception:
            # Fallback to unsmoothed aisle path if spline fails
            smooth_points = [(w[0], w[1], 0.0) for w in full_waypoints]
            max_curvature = 0.0
            path_length = aisle_distance

        # Step 6: Optionally refine start/end segments with Dubins
        waypoints = self._refine_endpoints_with_dubins(
            smooth_points, start_pose, goal_pose
        )

        stats = PlannerStats(
            path_length=path_length,
            computation_time=time.time() - start_time,
            num_waypoints=len(waypoints),
            max_curvature=max_curvature,
            path_type="astar+spline",
            num_aisle_nodes=len(self.factory_env.aisle_waypoints),
            astar_distance=aisle_distance,
        )

        return waypoints, stats

    def plan_to_station(
        self,
        start_pose: Tuple[float, float, float],
        station_id: str,
    ) -> Tuple[List[Tuple[float, float, float]], PlannerStats]:
        """Plan a path to a station's docking approach point.

        Args:
            start_pose: (x, y, theta) start configuration.
            station_id: ID of the target station.

        Returns:
            Tuple of (waypoints list, PlannerStats).
        """
        if station_id not in self.factory_env.stations:
            raise ValueError(f"Station {station_id} not found")

        station = self.factory_env.stations[station_id]
        approach_point = tuple(station["approach_point"])
        approach_heading = station.get("approach_heading", 0.0)

        goal_pose = (approach_point[0], approach_point[1], approach_heading)

        return self.plan(start_pose, goal_pose)

    def _astar_on_graph(
        self,
        start_node: str,
        goal_node: str,
    ) -> Optional[List[str]]:
        """A* search on the aisle waypoint graph.

        Args:
            start_node: Starting node ID.
            goal_node: Goal node ID.

        Returns:
            List of node IDs from start to goal, or None if no path exists.
        """
        if start_node not in self.factory_env.aisle_waypoints:
            return None
        if goal_node not in self.factory_env.aisle_waypoints:
            return None

        open_set: List[Tuple[float, str]] = [(0.0, start_node)]
        came_from: Dict[str, str] = {}
        g_score: Dict[str, float] = {start_node: 0.0}
        h_score: Dict[str, float] = {}

        # Precompute heuristic (straight-line distance to goal)
        goal_pos = self.factory_env.aisle_waypoints[goal_node]
        for node_id, node_pos in self.factory_env.aisle_waypoints.items():
            dx = node_pos[0] - goal_pos[0]
            dy = node_pos[1] - goal_pos[1]
            h_score[node_id] = math.sqrt(dx**2 + dy**2)

        closed_set: set = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_node:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            if current in closed_set:
                continue

            closed_set.add(current)

            # Explore neighbors
            if current not in self.factory_env.aisle_connections:
                continue

            for neighbor in self.factory_env.aisle_connections[current]:
                if neighbor in closed_set:
                    continue

                current_pos = self.factory_env.aisle_waypoints[current]
                neighbor_pos = self.factory_env.aisle_waypoints[neighbor]
                dx = neighbor_pos[0] - current_pos[0]
                dy = neighbor_pos[1] - current_pos[1]
                move_cost = math.sqrt(dx**2 + dy**2)

                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + h_score.get(neighbor, 0.0)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    def _find_nearest_waypoint(
        self,
        position: Tuple[float, float],
    ) -> Optional[str]:
        """Find the nearest aisle waypoint to a position.

        Args:
            position: (x, y) position.

        Returns:
            Node ID of nearest waypoint, or None if no waypoints exist.
        """
        if not self.factory_env.aisle_waypoints:
            return None

        min_dist = float("inf")
        nearest_node = None

        for node_id, waypoint in self.factory_env.aisle_waypoints.items():
            dx = waypoint[0] - position[0]
            dy = waypoint[1] - position[1]
            dist = math.sqrt(dx**2 + dy**2)

            if dist < min_dist:
                min_dist = dist
                nearest_node = node_id

        return nearest_node

    def _compute_path_distance(self, path: Optional[List[str]]) -> float:
        """Compute total Euclidean distance of a path in the aisle graph.

        Args:
            path: List of node IDs.

        Returns:
            Total path distance.
        """
        if not path or len(path) < 2:
            return 0.0

        total_dist = 0.0
        for i in range(len(path) - 1):
            node1 = path[i]
            node2 = path[i + 1]

            if node1 not in self.factory_env.aisle_waypoints:
                continue
            if node2 not in self.factory_env.aisle_waypoints:
                continue

            pos1 = self.factory_env.aisle_waypoints[node1]
            pos2 = self.factory_env.aisle_waypoints[node2]

            dx = pos2[0] - pos1[0]
            dy = pos2[1] - pos1[1]
            total_dist += math.sqrt(dx**2 + dy**2)

        return total_dist

    def _refine_endpoints_with_dubins(
        self,
        smooth_points: List[Tuple[float, float, float]],
        start_pose: Tuple[float, float, float],
        goal_pose: Tuple[float, float, float],
    ) -> List[Tuple[float, float, float]]:
        """Refine start/end segments with Dubins paths for heading awareness.

        Args:
            smooth_points: Smoothed waypoints from spline.
            start_pose: Start configuration (x, y, theta).
            goal_pose: Goal configuration (x, y, theta).

        Returns:
            Refined waypoint list with Dubins start/end segments.
        """
        if len(smooth_points) < 2:
            return smooth_points

        refined: List[Tuple[float, float, float]] = []

        # Start segment: Dubins from start_pose to first smooth point
        first_smooth = smooth_points[0]
        first_smooth_pose = (first_smooth[0], first_smooth[1], first_smooth[2])

        start_dubins = self.dubins_planner.plan(start_pose, first_smooth_pose)
        if start_dubins is not None:
            start_samples = self.dubins_planner.sample_path(start_dubins, start_pose)
            refined.extend(start_samples)
        else:
            refined.append(start_pose)

        # Middle: Use smooth points (skip first since it's in start segment)
        refined.extend(smooth_points[1:])

        # End segment: Dubins from last smooth point to goal
        last_smooth = smooth_points[-1]
        last_smooth_pose = (last_smooth[0], last_smooth[1], last_smooth[2])

        end_dubins = self.dubins_planner.plan(last_smooth_pose, goal_pose)
        if end_dubins is not None:
            end_samples = self.dubins_planner.sample_path(end_dubins, last_smooth_pose)
            # Skip first point to avoid duplication
            refined.extend(end_samples[1:])
        else:
            refined.append(goal_pose)

        return refined
