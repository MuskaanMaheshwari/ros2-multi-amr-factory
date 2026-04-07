"""Factory environment module for EV/battery manufacturing plant simulation.

This module simulates a realistic battery manufacturing section of an EV factory,
including station layout, zone management, and obstacle detection for autonomous
mobile robot (AMR) navigation.

Author: Muskaan Maheshwari
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple
import math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, Polygon, FancyArrowPatch


class StationType(Enum):
    """Types of stations in the factory environment."""

    INCOMING_MATERIAL = "incoming_material"
    CELL_ASSEMBLY = "cell_assembly"
    MODULE_PACKING = "module_packing"
    PACK_INTEGRATION = "pack_integration"
    TESTING_QC = "testing_qc"
    SHIPPING = "shipping"
    CHARGING_STATION = "charging_station"
    PARKING_BAY = "parking_bay"
    LOADING_DOCK = "loading_dock"
    UNLOADING_DOCK = "unloading_dock"


class FactoryZone(Enum):
    """Types of zones in the factory floor."""

    PRODUCTION = "production"
    AISLE = "aisle"
    INTERSECTION = "intersection"
    RESTRICTED = "restricted"
    BUFFER = "buffer"


@dataclass
class Station:
    """Represents a station on the factory floor.

    Attributes:
        station_id: Unique identifier for the station
        station_type: Type of station (StationType enum)
        position: Center position in meters (x, y)
        orientation: Heading in radians (0 = east, π/2 = north)
        width: Station width in meters
        length: Station length in meters
        is_occupied: Whether station is currently occupied
        occupied_by: Robot ID occupying the station (if occupied)
        approach_point: Point 2m in front for approach navigation
        approach_heading: Heading robot should have when approaching
    """

    station_id: str
    station_type: StationType
    position: Tuple[float, float]
    orientation: float
    width: float
    length: float
    is_occupied: bool = False
    occupied_by: Optional[str] = None
    approach_point: Tuple[float, float] = field(default=(0.0, 0.0))
    approach_heading: float = field(default=0.0)

    def __post_init__(self):
        """Calculate approach point and heading after initialization."""
        if self.approach_point == (0.0, 0.0):
            # Calculate approach point 2m in front of station
            approach_distance = 2.0
            x, y = self.position
            approach_x = x - approach_distance * math.cos(self.orientation)
            approach_y = y - approach_distance * math.sin(self.orientation)
            object.__setattr__(self, "approach_point", (approach_x, approach_y))
            object.__setattr__(self, "approach_heading", self.orientation)


class FactoryEnvironment:
    """Simulates a realistic EV/battery manufacturing factory floor.

    The environment represents approximately 100m x 80m factory floor with:
    - Material receiving docks (west wall)
    - Battery assembly and packing stations (middle rows)
    - Pack integration line
    - Testing/QC bays
    - Shipping docks (east wall)
    - Charging and parking areas for AMRs
    - Aisles and intersections for navigation
    - Restricted zones

    Attributes:
        width: Factory floor width in meters
        height: Factory floor height in meters
        resolution: Meters per grid cell for obstacle detection
    """

    def __init__(self, width: float = 100.0, height: float = 80.0, resolution: float = 0.5):
        """Initialize the factory environment.

        Args:
            width: Factory floor width in meters
            height: Factory floor height in meters
            resolution: Meters per grid cell for obstacle detection
        """
        self.width = width
        self.height = height
        self.resolution = resolution

        # Stations indexed by station_id
        self.stations: Dict[str, Station] = {}

        # Zone grid: stores FactoryZone for each cell
        self.zone_grid = np.full(
            (int(height / resolution), int(width / resolution)),
            FactoryZone.BUFFER,
            dtype=object,
        )

        # Obstacle grid: binary (1 = obstacle, 0 = free)
        self.obstacle_grid = np.zeros(
            (int(height / resolution), int(width / resolution)), dtype=np.uint8
        )

        # Aisle waypoints and graph
        self.aisle_waypoints: List[Tuple[float, float]] = []
        self.aisle_graph: Dict[int, List[int]] = {}

        # Build the factory layout
        self._build_ev_battery_layout()
        self._create_obstacle_grid()
        self._build_aisle_graph()

    def _build_ev_battery_layout(self) -> None:
        """Build realistic EV battery factory layout.

        Layout structure:
        - West wall: 4 incoming material docks
        - Row 1: 6 cell assembly stations (2x3 grid)
        - Row 2: 4 module packing stations
        - Row 3: 3 pack integration stations (larger)
        - Row 4: 4 testing/QC bays
        - East wall: 3 shipping docks
        - South wall: 4 charging stations
        - North wall: 6 parking bays
        - Aisles: 4m wide horizontal, 3m wide vertical
        """
        station_counter = 0

        # West wall: Incoming material docks
        incoming_y_positions = [10.0, 25.0, 55.0, 70.0]
        for idx, y in enumerate(incoming_y_positions):
            station_id = f"incoming_{idx}"
            self.stations[station_id] = Station(
                station_id=station_id,
                station_type=StationType.INCOMING_MATERIAL,
                position=(5.0, y),
                orientation=0.0,  # Facing east
                width=3.0,
                length=4.0,
            )

        # Row 1: Cell assembly stations (x=15-30, 2 rows of 3)
        cell_assembly_x = [15.0, 22.5, 30.0]
        cell_assembly_y = [20.0, 60.0]
        counter = 0
        for x in cell_assembly_x:
            for y in cell_assembly_y:
                station_id = f"cell_assembly_{counter}"
                self.stations[station_id] = Station(
                    station_id=station_id,
                    station_type=StationType.CELL_ASSEMBLY,
                    position=(x, y),
                    orientation=math.pi / 2,  # Facing north
                    width=4.5,
                    length=5.0,
                )
                counter += 1

        # Row 2: Module packing stations (x=40, 2 rows of 2)
        module_packing_x = [40.0, 47.0]
        module_packing_y = [22.0, 58.0]
        counter = 0
        for x in module_packing_x:
            for y in module_packing_y:
                station_id = f"module_packing_{counter}"
                self.stations[station_id] = Station(
                    station_id=station_id,
                    station_type=StationType.MODULE_PACKING,
                    position=(x, y),
                    orientation=0.0,  # Facing east
                    width=5.0,
                    length=4.5,
                )
                counter += 1

        # Row 3: Pack integration stations (larger, x=63, 3 stations)
        pack_integration_y = [18.0, 40.0, 62.0]
        counter = 0
        for y in pack_integration_y:
            station_id = f"pack_integration_{counter}"
            self.stations[station_id] = Station(
                station_id=station_id,
                station_type=StationType.PACK_INTEGRATION,
                position=(63.0, y),
                orientation=math.pi / 2,  # Facing north
                width=8.0,
                length=5.0,
            )
            counter += 1

        # Row 4: Testing/QC bays (x=80, 4 bays)
        testing_y = [15.0, 35.0, 45.0, 65.0]
        counter = 0
        for y in testing_y:
            station_id = f"testing_qc_{counter}"
            self.stations[station_id] = Station(
                station_id=station_id,
                station_type=StationType.TESTING_QC,
                position=(80.0, y),
                orientation=0.0,  # Facing east
                width=6.0,
                length=4.5,
            )
            counter += 1

        # East wall: Shipping docks
        shipping_y = [20.0, 40.0, 60.0]
        for idx, y in enumerate(shipping_y):
            station_id = f"shipping_{idx}"
            self.stations[station_id] = Station(
                station_id=station_id,
                station_type=StationType.SHIPPING,
                position=(95.0, y),
                orientation=math.pi,  # Facing west
                width=3.0,
                length=4.0,
            )

        # South wall: Charging stations (facing north)
        charging_x = [20.0, 40.0, 60.0, 80.0]
        for idx, x in enumerate(charging_x):
            station_id = f"charging_{idx}"
            self.stations[station_id] = Station(
                station_id=station_id,
                station_type=StationType.CHARGING_STATION,
                position=(x, 2.0),
                orientation=math.pi / 2,  # Facing north
                width=3.0,
                length=2.5,
            )

        # North wall: Parking bays (facing south)
        parking_x = [15.0, 30.0, 45.0, 60.0, 75.0, 90.0]
        for idx, x in enumerate(parking_x):
            station_id = f"parking_{idx}"
            self.stations[station_id] = Station(
                station_id=station_id,
                station_type=StationType.PARKING_BAY,
                position=(x, 78.0),
                orientation=3 * math.pi / 2,  # Facing south
                width=3.0,
                length=2.5,
            )

    def _create_obstacle_grid(self) -> None:
        """Build binary obstacle grid from station footprints and walls."""
        # Mark all cells as free initially (except will set to production/buffer)
        grid_height, grid_width = self.obstacle_grid.shape

        # Initialize zone grid as BUFFER
        self.zone_grid[:, :] = FactoryZone.BUFFER

        # Add station obstacles
        for station in self.stations.values():
            x, y = station.position
            w, h = station.width, station.length

            # Calculate corners
            corners = [
                (x - w / 2, y - h / 2),
                (x + w / 2, y - h / 2),
                (x + w / 2, y + h / 2),
                (x - w / 2, y + h / 2),
            ]

            # Mark cells as obstacles
            for cx in np.arange(x - w / 2, x + w / 2, self.resolution):
                for cy in np.arange(y - h / 2, y + h / 2, self.resolution):
                    grid_x = int(cx / self.resolution)
                    grid_y = int(cy / self.resolution)
                    if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                        self.obstacle_grid[grid_y, grid_x] = 1
                        self.zone_grid[grid_y, grid_x] = FactoryZone.PRODUCTION

        # Mark aisles
        # Horizontal aisles: 4m wide at y = 33, 33+4.5 = 37.5, ...
        aisle_y_positions = [10.0, 20.0, 35.0, 50.0, 65.0, 75.0]
        aisle_width = 4.0

        for aisle_y in aisle_y_positions:
            for cy in np.arange(aisle_y - aisle_width / 2, aisle_y + aisle_width / 2, self.resolution):
                for cx in np.arange(0, self.width, self.resolution):
                    grid_x = int(cx / self.resolution)
                    grid_y = int(cy / self.resolution)
                    if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                        if self.obstacle_grid[grid_y, grid_x] == 0:
                            self.zone_grid[grid_y, grid_x] = FactoryZone.AISLE

        # Vertical aisles: 3m wide at x = 12, 37, 58, 77
        aisle_x_positions = [12.0, 37.0, 58.0, 77.0]
        aisle_width = 3.0

        for aisle_x in aisle_x_positions:
            for cx in np.arange(aisle_x - aisle_width / 2, aisle_x + aisle_width / 2, self.resolution):
                for cy in np.arange(0, self.height, self.resolution):
                    grid_x = int(cx / self.resolution)
                    grid_y = int(cy / self.resolution)
                    if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                        if self.obstacle_grid[grid_y, grid_x] == 0:
                            self.zone_grid[grid_y, grid_x] = FactoryZone.AISLE

        # Mark intersections (where aisles cross)
        h_aisles = [10.0, 20.0, 35.0, 50.0, 65.0, 75.0]
        v_aisles = [12.0, 37.0, 58.0, 77.0]
        intersection_size = 1.5

        for hx in h_aisles:
            for vy in v_aisles:
                for cx in np.arange(vy - intersection_size, vy + intersection_size, self.resolution):
                    for cy in np.arange(hx - intersection_size, hx + intersection_size, self.resolution):
                        grid_x = int(cx / self.resolution)
                        grid_y = int(cy / self.resolution)
                        if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                            if self.obstacle_grid[grid_y, grid_x] == 0:
                                self.zone_grid[grid_y, grid_x] = FactoryZone.INTERSECTION

        # Restricted zones at 4 corners (emergency exits)
        restricted_size = 4.0
        corners_restrict = [
            (0, 0),  # SW
            (self.width, 0),  # SE
            (0, self.height),  # NW
            (self.width, self.height),  # NE
        ]

        for corner_x, corner_y in corners_restrict:
            for cx in np.arange(max(0, corner_x - restricted_size), min(self.width, corner_x + restricted_size), self.resolution):
                for cy in np.arange(max(0, corner_y - restricted_size), min(self.height, corner_y + restricted_size), self.resolution):
                    grid_x = int(cx / self.resolution)
                    grid_y = int(cy / self.resolution)
                    if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                        self.obstacle_grid[grid_y, grid_x] = 1
                        self.zone_grid[grid_y, grid_x] = FactoryZone.RESTRICTED

    def _build_aisle_graph(self) -> None:
        """Build waypoint graph for aisle navigation.

        Creates waypoints at aisle intersections and along main paths.
        """
        # Define horizontal aisle positions and vertical aisle positions
        h_aisles = [10.0, 20.0, 35.0, 50.0, 65.0, 75.0]
        v_aisles = [12.0, 37.0, 58.0, 77.0]

        # Create waypoints at intersections
        waypoint_map = {}
        waypoint_idx = 0

        for hx in h_aisles:
            for vy in v_aisles:
                waypoint_map[(hx, vy)] = waypoint_idx
                self.aisle_waypoints.append((vy, hx))
                waypoint_idx += 1

        # Additional waypoints along aisles (ends)
        for hx in h_aisles:
            waypoint_map[(hx, 0)] = waypoint_idx
            self.aisle_waypoints.append((0.0, hx))
            waypoint_idx += 1

            waypoint_map[(hx, self.width)] = waypoint_idx
            self.aisle_waypoints.append((self.width, hx))
            waypoint_idx += 1

        for vy in v_aisles:
            waypoint_map[(0, vy)] = waypoint_idx
            self.aisle_waypoints.append((vy, 0.0))
            waypoint_idx += 1

            waypoint_map[(self.height, vy)] = waypoint_idx
            self.aisle_waypoints.append((vy, self.height))
            waypoint_idx += 1

        # Build adjacency graph
        self.aisle_graph = {i: [] for i in range(len(self.aisle_waypoints))}

        # Connect intersections along horizontal aisles
        for hx in h_aisles:
            indices = [waypoint_map[(hx, vy)] for vy in sorted(v_aisles)]
            for i in range(len(indices) - 1):
                self.aisle_graph[indices[i]].append(indices[i + 1])
                self.aisle_graph[indices[i + 1]].append(indices[i])

        # Connect intersections along vertical aisles
        for vy in v_aisles:
            indices = [waypoint_map[(hx, vy)] for hx in sorted(h_aisles)]
            for i in range(len(indices) - 1):
                self.aisle_graph[indices[i]].append(indices[i + 1])
                self.aisle_graph[indices[i + 1]].append(indices[i])

    def get_stations_by_type(self, station_type: StationType) -> List[Station]:
        """Get all stations of a given type.

        Args:
            station_type: The type of station to filter by

        Returns:
            List of stations matching the type
        """
        return [s for s in self.stations.values() if s.station_type == station_type]

    def get_nearest_available_station(
        self, position: Tuple[float, float], station_type: StationType
    ) -> Optional[Station]:
        """Get the nearest unoccupied station of a given type.

        Args:
            position: Current position (x, y) in meters
            station_type: Type of station to find

        Returns:
            Nearest available station, or None if none available
        """
        available = [s for s in self.get_stations_by_type(station_type) if not s.is_occupied]

        if not available:
            return None

        nearest = min(
            available,
            key=lambda s: math.sqrt((s.position[0] - position[0]) ** 2 + (s.position[1] - position[1]) ** 2),
        )

        return nearest

    def get_aisle_graph(self) -> Dict[int, List[int]]:
        """Get the aisle navigation graph.

        Returns:
            Dictionary mapping waypoint indices to lists of connected waypoint indices
        """
        return self.aisle_graph.copy()

    def is_in_zone(self, position: Tuple[float, float], zone_type: FactoryZone) -> bool:
        """Check if a position is in a specific zone type.

        Args:
            position: Position (x, y) in meters
            zone_type: Zone type to check

        Returns:
            True if position is in the zone type
        """
        x, y = position
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)

        if not (0 <= grid_x < self.obstacle_grid.shape[1] and 0 <= grid_y < self.obstacle_grid.shape[0]):
            return False

        return self.zone_grid[grid_y, grid_x] == zone_type

    def get_intersections(self) -> List[Tuple[float, float]]:
        """Get all intersection centers for traffic management.

        Returns:
            List of intersection center positions (x, y)
        """
        h_aisles = [10.0, 20.0, 35.0, 50.0, 65.0, 75.0]
        v_aisles = [12.0, 37.0, 58.0, 77.0]

        intersections = []
        for hx in h_aisles:
            for vy in v_aisles:
                intersections.append((vy, hx))

        return intersections

    def occupy_station(self, station_id: str, robot_id: str) -> bool:
        """Mark a station as occupied.

        Args:
            station_id: ID of the station
            robot_id: ID of the robot occupying it

        Returns:
            True if successful, False if station doesn't exist or already occupied
        """
        if station_id not in self.stations:
            return False

        station = self.stations[station_id]
        if station.is_occupied:
            return False

        station.is_occupied = True
        station.occupied_by = robot_id
        return True

    def release_station(self, station_id: str) -> bool:
        """Release a station (mark as unoccupied).

        Args:
            station_id: ID of the station to release

        Returns:
            True if successful, False if station doesn't exist
        """
        if station_id not in self.stations:
            return False

        station = self.stations[station_id]
        station.is_occupied = False
        station.occupied_by = None
        return True

    def visualize(
        self,
        robots: Optional[List[Dict]] = None,
        paths: Optional[List[List[Tuple[float, float]]]] = None,
        title: Optional[str] = None,
    ) -> plt.Figure:
        """Visualize the factory environment.

        Args:
            robots: Optional list of robot dicts with keys:
                - 'id': robot identifier
                - 'position': (x, y) current position
                - 'heading': heading in radians
                - 'color': color string (optional)
            paths: Optional list of paths, each path is a list of (x, y) waypoints
            title: Optional title for the figure

        Returns:
            matplotlib Figure object
        """
        fig, ax = plt.subplots(figsize=(14, 10))

        # Set up axes
        ax.set_xlim(0, self.width)
        ax.set_ylim(0, self.height)
        ax.set_aspect("equal")
        ax.invert_yaxis()
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")

        if title:
            ax.set_title(title, fontsize=14, fontweight="bold")
        else:
            ax.set_title("EV Battery Factory Floor", fontsize=14, fontweight="bold")

        # Define colors for station types
        station_colors = {
            StationType.INCOMING_MATERIAL: "#FF6B6B",
            StationType.CELL_ASSEMBLY: "#4ECDC4",
            StationType.MODULE_PACKING: "#45B7D1",
            StationType.PACK_INTEGRATION: "#96CEB4",
            StationType.TESTING_QC: "#FFEAA7",
            StationType.SHIPPING: "#DDA15E",
            StationType.CHARGING_STATION: "#BC6C25",
            StationType.PARKING_BAY: "#8E7DBE",
        }

        # Draw aisles (light gray)
        for y in [10.0, 20.0, 35.0, 50.0, 65.0, 75.0]:
            ax.add_patch(
                patches.Rectangle((0, y - 2.0), self.width, 4.0, linewidth=0, facecolor="#E8E8E8", zorder=1)
            )

        for x in [12.0, 37.0, 58.0, 77.0]:
            ax.add_patch(patches.Rectangle((x - 1.5, 0), 3.0, self.height, linewidth=0, facecolor="#E8E8E8", zorder=1))

        # Draw restricted zones (red hatch)
        restricted_size = 4.0
        corners_restrict = [
            (0, 0),
            (self.width - restricted_size, 0),
            (0, self.height - restricted_size),
            (self.width - restricted_size, self.height - restricted_size),
        ]

        for cx, cy in corners_restrict:
            ax.add_patch(
                patches.Rectangle(
                    (cx, cy),
                    restricted_size,
                    restricted_size,
                    linewidth=1,
                    facecolor="red",
                    alpha=0.3,
                    hatch="///",
                    zorder=2,
                )
            )

        # Draw intersections (yellow diamonds)
        for ix, iy in self.get_intersections():
            ax.plot(ix, iy, marker="D", markersize=8, color="yellow", markeredgecolor="black", zorder=3)

        # Draw stations
        for station in self.stations.values():
            x, y = station.position
            w, h = station.width, station.length

            color = station_colors.get(station.station_type, "#CCCCCC")
            alpha = 0.7 if not station.is_occupied else 0.95
            edge_color = "darkred" if station.is_occupied else "black"
            edge_width = 2 if station.is_occupied else 1

            # Draw station rectangle
            rect = FancyBboxPatch(
                (x - w / 2, y - h / 2),
                w,
                h,
                boxstyle="round,pad=0.05",
                linewidth=edge_width,
                edgecolor=edge_color,
                facecolor=color,
                alpha=alpha,
                zorder=4,
            )
            ax.add_patch(rect)

            # Draw approach point
            ax_pt = station.approach_point
            ax.plot(ax_pt[0], ax_pt[1], "o", color="green", markersize=3, zorder=5)

            # Draw orientation arrow
            arrow_len = 1.5
            dx = arrow_len * math.cos(station.orientation)
            dy = arrow_len * math.sin(station.orientation)
            ax.arrow(
                x,
                y,
                dx,
                dy,
                head_width=0.3,
                head_length=0.2,
                fc="black",
                ec="black",
                alpha=0.6,
                zorder=5,
            )

            # Add station ID label
            ax.text(x, y, station.station_id.replace("_", "\n"), ha="center", va="center", fontsize=6, zorder=6)

        # Draw charging stations with lightning symbol
        for station in self.get_stations_by_type(StationType.CHARGING_STATION):
            ax.text(
                station.position[0],
                station.position[1],
                "⚡",
                ha="center",
                va="center",
                fontsize=12,
                zorder=7,
            )

        # Draw parking bays with P marker
        for station in self.get_stations_by_type(StationType.PARKING_BAY):
            ax.text(
                station.position[0],
                station.position[1],
                "P",
                ha="center",
                va="center",
                fontsize=10,
                fontweight="bold",
                color="white",
                zorder=7,
            )

        # Draw robots if provided
        if robots:
            for robot in robots:
                rx, ry = robot["position"]
                rh = robot["heading"]
                color = robot.get("color", "#2E86AB")

                # Draw robot circle
                circle = patches.Circle((rx, ry), 0.4, color=color, ec="black", linewidth=2, zorder=8)
                ax.add_patch(circle)

                # Draw heading arrow
                arrow_len = 0.6
                dx = arrow_len * math.cos(rh)
                dy = arrow_len * math.sin(rh)
                ax.arrow(rx, ry, dx, dy, head_width=0.2, head_length=0.15, fc="white", ec="white", zorder=9)

                # Add robot ID label
                ax.text(rx, ry - 0.7, robot.get("id", "?"), ha="center", fontsize=8, fontweight="bold", zorder=10)

        # Draw paths if provided
        if paths:
            colors = plt.cm.rainbow(np.linspace(0, 1, len(paths)))
            for path, path_color in zip(paths, colors):
                if len(path) > 1:
                    path_x = [p[0] for p in path]
                    path_y = [p[1] for p in path]
                    ax.plot(path_x, path_y, "--", color=path_color, linewidth=2, alpha=0.6, zorder=3)
                    ax.plot(path_x[0], path_y[0], "o", color=path_color, markersize=6, zorder=3)
                    ax.plot(path_x[-1], path_y[-1], "s", color=path_color, markersize=6, zorder=3)

        # Legend
        legend_elements = [
            patches.Patch(facecolor=station_colors[StationType.INCOMING_MATERIAL], label="Incoming Material"),
            patches.Patch(facecolor=station_colors[StationType.CELL_ASSEMBLY], label="Cell Assembly"),
            patches.Patch(facecolor=station_colors[StationType.MODULE_PACKING], label="Module Packing"),
            patches.Patch(facecolor=station_colors[StationType.PACK_INTEGRATION], label="Pack Integration"),
            patches.Patch(facecolor=station_colors[StationType.TESTING_QC], label="Testing/QC"),
            patches.Patch(facecolor=station_colors[StationType.SHIPPING], label="Shipping"),
            patches.Patch(facecolor=station_colors[StationType.CHARGING_STATION], label="Charging"),
            patches.Patch(facecolor=station_colors[StationType.PARKING_BAY], label="Parking"),
            patches.Patch(facecolor="#E8E8E8", label="Aisle"),
            patches.Patch(facecolor="red", alpha=0.3, label="Restricted"),
            plt.Line2D([0], [0], marker="D", color="w", markerfacecolor="yellow", markeredgecolor="black", markersize=8, label="Intersection"),
        ]

        ax.legend(
            handles=legend_elements,
            loc="upper left",
            bbox_to_anchor=(1.01, 1.0),
            fontsize=8,
            framealpha=0.95,
            borderaxespad=0,
        )

        # Grid
        ax.grid(True, alpha=0.2, linestyle=":")

        fig.tight_layout(rect=[0, 0, 0.85, 1])
        return fig
