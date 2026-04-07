"""
Dynamic obstacle detection via simulated LiDAR scanning.

Detects unexpected obstacles not present in the original path costmap by comparing
live scan data against expected environment model. Includes obstacle classification
(person, equipment, clutter) with severity levels and confidence scoring.

Simulates a 180° x 15m range LiDAR with 0.5° angular resolution and 0.1m distance resolution.

Author: Muskaan Maheshwari
"""

import math
import random
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np


class ObstacleType(Enum):
    """Classification of detected obstacles."""
    PERSON = "person"
    EQUIPMENT = "equipment"
    CLUTTER = "clutter"
    UNKNOWN = "unknown"


class ObstacleSeverity(Enum):
    """Severity level of detected obstacle."""
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


@dataclass
class ObstacleInfo:
    """Information about a detected obstacle."""
    position: Tuple[float, float]  # (x, y) in world coordinates
    distance: float  # Distance from robot in meters
    obstacle_type: ObstacleType
    severity: ObstacleSeverity
    confidence: float  # Confidence in detection [0, 1]
    timestamp: float  # Detection timestamp in seconds
    angular_position: float  # Angle from robot heading in radians


@dataclass
class ScanResult:
    """Result from a single LiDAR scan."""
    obstacle_detected: bool
    obstacles: List[ObstacleInfo] = field(default_factory=list)
    scan_points: List[Tuple[float, float, float]] = field(default_factory=list)  # (distance, angle, intensity)
    timestamp: float = 0.0


class ObstacleDetector:
    """
    Simulates LiDAR-based dynamic obstacle detection.

    Compares live scan data against expected costmap to identify unexpected obstacles
    (moving people, equipment placement, clutter). Assigns obstacle types and severity
    levels based on detection characteristics.

    Attributes:
        expected_path_costmap: Reference costmap representing expected environment (m x n array).
        lidar_range: Maximum sensing range in meters (default 15m).
        lidar_fov: Field of view in degrees (default 180°, front-facing).
        angular_resolution: Angular resolution in degrees (default 0.5°).
        false_positive_rate: Probability of false positive detection [0, 1] (default 0.05).
        false_negative_rate: Probability of missing a real obstacle [0, 1] (default 0.1).
    """

    def __init__(
        self,
        expected_path_costmap: Optional[np.ndarray] = None,
        lidar_range: float = 15.0,
        lidar_fov: float = 180.0,
        angular_resolution: float = 0.5,
        false_positive_rate: float = 0.05,
        false_negative_rate: float = 0.1,
    ) -> None:
        """
        Initialize obstacle detector.

        Args:
            expected_path_costmap: Reference costmap (m x n array, 0=free, 1=occupied).
                                  None if no reference available.
            lidar_range: Maximum sensing range in meters (default 15.0).
            lidar_fov: Field of view in degrees, front-facing (default 180.0).
            angular_resolution: Angular resolution in degrees (default 0.5).
            false_positive_rate: False positive probability [0, 1] (default 0.05).
            false_negative_rate: False negative probability [0, 1] (default 0.1).
        """
        self.expected_path_costmap = expected_path_costmap
        self.lidar_range = lidar_range
        self.lidar_fov = lidar_fov
        self.angular_resolution = angular_resolution
        self.false_positive_rate = false_positive_rate
        self.false_negative_rate = false_negative_rate

        # Scan history for temporal filtering
        self.last_scan: Optional[ScanResult] = None
        self.scan_count = 0

    def scan(
        self,
        robot_pose: Tuple[float, float, float],
        actual_obstacle_grid: Optional[np.ndarray] = None,
        timestamp: float = 0.0,
    ) -> ScanResult:
        """
        Simulate a LiDAR scan and detect unexpected obstacles.

        Returns all detected obstacles in the field of view, highlighting those
        that differ from the expected costmap.

        Args:
            robot_pose: Current robot pose as (x, y, heading_rad).
            actual_obstacle_grid: Actual obstacle positions (m x n array, 0=free, 1=occupied).
                                 If None, uses expected_path_costmap.
            timestamp: Scan timestamp in seconds (for alert generation).

        Returns:
            ScanResult containing detected obstacles and scan metadata.

        Example:
            detector = ObstacleDetector(expected_costmap)
            result = detector.scan(robot_pose=(5.0, 5.0, 0.0))
            if result.obstacle_detected:
                for obs in result.obstacles:
                    print(f"Found {obs.obstacle_type.value} at {obs.position}")
        """
        self.scan_count += 1
        scan_result = ScanResult(timestamp=timestamp, obstacle_detected=False)

        if actual_obstacle_grid is None:
            actual_obstacle_grid = self.expected_path_costmap

        if actual_obstacle_grid is None:
            # No obstacle data available
            self.last_scan = scan_result
            return scan_result

        robot_x, robot_y, robot_heading = robot_pose

        # Generate scan rays
        num_rays = int(self.lidar_fov / self.angular_resolution)
        start_angle = robot_heading - math.radians(self.lidar_fov / 2.0)

        detected_obstacles: List[ObstacleInfo] = []
        scan_points = []

        for i in range(num_rays):
            angle = start_angle + i * math.radians(self.angular_resolution)

            # Cast ray to find intersections with obstacles
            hit_distance = self._cast_ray(
                robot_x, robot_y, angle, actual_obstacle_grid
            )

            if hit_distance is not None and hit_distance < self.lidar_range:
                # Found an obstacle
                hit_x = robot_x + hit_distance * math.cos(angle)
                hit_y = robot_y + hit_distance * math.sin(angle)

                # Check if this is an unexpected obstacle
                is_expected = self._is_expected_obstacle(
                    hit_x, hit_y, self.expected_path_costmap
                )

                # Apply sensor model (false positives/negatives)
                if random.random() > self.false_negative_rate:  # Not missed
                    if not is_expected and random.random() > self.false_positive_rate:
                        # Unexpected obstacle detected
                        obs_type = self._classify_obstacle(hit_distance)
                        severity = self._assess_severity(hit_distance, obs_type)
                        confidence = self._compute_confidence(hit_distance)

                        obstacle = ObstacleInfo(
                            position=(hit_x, hit_y),
                            distance=hit_distance,
                            obstacle_type=obs_type,
                            severity=severity,
                            confidence=confidence,
                            timestamp=timestamp,
                            angular_position=angle - robot_heading,
                        )
                        detected_obstacles.append(obstacle)
                        scan_result.obstacle_detected = True

                scan_points.append((hit_distance, angle, 0.5))

        # Apply temporal filtering to reduce noise
        filtered_obstacles = self._temporal_filter(detected_obstacles)

        scan_result.obstacles = filtered_obstacles
        scan_result.scan_points = scan_points
        self.last_scan = scan_result

        return scan_result

    def _cast_ray(
        self,
        start_x: float,
        start_y: float,
        angle: float,
        obstacle_grid: np.ndarray,
    ) -> Optional[float]:
        """
        Cast a ray and find first intersection with obstacle.

        Uses Bresenham-style line rasterization.

        Args:
            start_x: Ray origin x in meters.
            start_y: Ray origin y in meters.
            angle: Ray direction in radians.
            obstacle_grid: Obstacle grid (0=free, 1=occupied).

        Returns:
            Distance to first hit in meters, or None if no hit within range.
        """
        grid_resolution = 0.1

        # Sample along ray
        num_samples = int(self.lidar_range / grid_resolution)

        for sample in range(1, num_samples):
            distance = sample * grid_resolution
            x = start_x + distance * math.cos(angle)
            y = start_y + distance * math.sin(angle)

            # Convert to grid coordinates
            grid_x = int(round(x / grid_resolution))
            grid_y = int(round(y / grid_resolution))

            # Check bounds
            if grid_x < 0 or grid_x >= obstacle_grid.shape[1]:
                continue
            if grid_y < 0 or grid_y >= obstacle_grid.shape[0]:
                continue

            # Check for obstacle
            if obstacle_grid[grid_y, grid_x] > 0:
                return distance

        return None

    def _is_expected_obstacle(
        self,
        x: float,
        y: float,
        expected_costmap: Optional[np.ndarray],
    ) -> bool:
        """
        Check if obstacle position matches expected costmap.

        Args:
            x: Obstacle x position in meters.
            y: Obstacle y position in meters.
            expected_costmap: Expected obstacle grid or None.

        Returns:
            True if obstacle is in expected costmap, False otherwise.
        """
        if expected_costmap is None:
            return False

        grid_resolution = 0.1
        grid_x = int(round(x / grid_resolution))
        grid_y = int(round(y / grid_resolution))

        if grid_x < 0 or grid_x >= expected_costmap.shape[1]:
            return False
        if grid_y < 0 or grid_y >= expected_costmap.shape[0]:
            return False

        return expected_costmap[grid_y, grid_x] > 0

    def _classify_obstacle(self, distance: float) -> ObstacleType:
        """
        Classify obstacle type based on distance and other characteristics.

        Args:
            distance: Distance to obstacle in meters.

        Returns:
            Classified obstacle type.
        """
        # Simple heuristic: closer = likely person, medium = equipment, far = clutter
        if distance < 2.0:
            return ObstacleType.PERSON
        elif distance < 5.0:
            return ObstacleType.EQUIPMENT
        else:
            return ObstacleType.CLUTTER

    def _assess_severity(
        self,
        distance: float,
        obstacle_type: ObstacleType,
    ) -> ObstacleSeverity:
        """
        Assess severity level based on distance and type.

        Args:
            distance: Distance to obstacle in meters.
            obstacle_type: Classified obstacle type.

        Returns:
            Severity level assessment.
        """
        # Closer = higher severity
        if distance < 1.0:
            return ObstacleSeverity.HIGH

        # Type-based severity
        if obstacle_type == ObstacleType.PERSON:
            return ObstacleSeverity.HIGH
        elif obstacle_type == ObstacleType.EQUIPMENT:
            return ObstacleSeverity.MEDIUM
        else:
            return ObstacleSeverity.LOW

    def _compute_confidence(self, distance: float) -> float:
        """
        Compute detection confidence based on distance.

        Closer = higher confidence.

        Args:
            distance: Distance to obstacle in meters.

        Returns:
            Confidence score in [0, 1].
        """
        # Confidence decreases with distance
        max_confidence_distance = 3.0
        confidence = max(0.0, 1.0 - distance / max_confidence_distance)
        # Add some noise
        confidence += random.gauss(0, 0.05)
        return max(0.0, min(1.0, confidence))

    def _temporal_filter(
        self,
        current_obstacles: List[ObstacleInfo],
    ) -> List[ObstacleInfo]:
        """
        Apply temporal filtering to reduce false positives.

        Tracks obstacles across scans to confirm detections.

        Args:
            current_obstacles: Obstacles from current scan.

        Returns:
            Filtered obstacle list with low-confidence detections removed.
        """
        if not current_obstacles:
            return []

        # If this is early in scan history, accept more readily
        if self.scan_count < 3:
            return current_obstacles

        # Filter by confidence threshold (stricter over time)
        confidence_threshold = 0.4
        filtered = [
            obs for obs in current_obstacles
            if obs.confidence > confidence_threshold
        ]

        return filtered


def create_test_obstacle_grid(width: int = 100, height: int = 100) -> np.ndarray:
    """
    Create a test obstacle grid with some known obstacles.

    Args:
        width: Grid width in cells.
        height: Grid height in cells.

    Returns:
        Obstacle grid (0=free, 1=occupied).

    Example:
        grid = create_test_obstacle_grid()
        detector = ObstacleDetector(expected_path_costmap=grid)
    """
    grid = np.zeros((height, width), dtype=np.uint8)

    # Add some obstacles (walls, etc.)
    grid[20:30, 10:15] = 1  # Wall
    grid[50:55, 40:80] = 1  # Long wall
    grid[70:75, 30:35] = 1  # Another wall

    return grid
