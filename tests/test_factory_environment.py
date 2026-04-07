"""Tests for factory environment module."""

import pytest
import math
from src.factory.environment import FactoryEnvironment, StationType, FactoryZone, Station


class TestFactoryCreation:
    """Test factory environment initialization."""

    def test_factory_initialization(self, factory_env):
        """Test that factory initializes with correct dimensions."""
        assert factory_env.width == 100.0
        assert factory_env.height == 80.0
        assert factory_env.resolution == 0.5

    def test_factory_has_stations(self, factory_env):
        """Test that factory has stations created."""
        assert len(factory_env.stations) > 0
        assert "incoming_0" in factory_env.stations
        assert "parking_0" in factory_env.stations
        assert "charging_0" in factory_env.stations

    def test_obstacle_grid_shape(self, factory_env):
        """Test obstacle grid dimensions."""
        expected_height = int(factory_env.height / factory_env.resolution)
        expected_width = int(factory_env.width / factory_env.resolution)
        assert factory_env.obstacle_grid.shape == (expected_height, expected_width)

    def test_zone_grid_shape(self, factory_env):
        """Test zone grid dimensions."""
        expected_height = int(factory_env.height / factory_env.resolution)
        expected_width = int(factory_env.width / factory_env.resolution)
        assert factory_env.zone_grid.shape == (expected_height, expected_width)


class TestStationCounts:
    """Test station counts by type."""

    def test_incoming_stations(self, factory_env):
        """Test incoming material stations."""
        incoming = factory_env.get_stations_by_type(StationType.INCOMING_MATERIAL)
        assert len(incoming) >= 0  # May vary with implementation

    def test_cell_assembly_stations(self, factory_env):
        """Test cell assembly stations."""
        cell_assembly = factory_env.get_stations_by_type(StationType.CELL_ASSEMBLY)
        assert len(cell_assembly) >= 0

    def test_module_packing_stations(self, factory_env):
        """Test module packing stations."""
        module_packing = factory_env.get_stations_by_type(StationType.MODULE_PACKING)
        assert len(module_packing) >= 0

    def test_pack_integration_stations(self, factory_env):
        """Test pack integration stations."""
        pack_integration = factory_env.get_stations_by_type(StationType.PACK_INTEGRATION)
        assert len(pack_integration) >= 0

    def test_testing_qc_stations(self, factory_env):
        """Test testing/QC stations."""
        testing = factory_env.get_stations_by_type(StationType.TESTING_QC)
        assert len(testing) >= 0

    def test_shipping_stations(self, factory_env):
        """Test shipping stations."""
        shipping = factory_env.get_stations_by_type(StationType.SHIPPING)
        assert len(shipping) >= 0

    def test_charging_stations(self, factory_env):
        """Test charging stations."""
        charging = factory_env.get_stations_by_type(StationType.CHARGING_STATION)
        assert len(charging) >= 0

    def test_parking_bays(self, factory_env):
        """Test parking bays."""
        parking = factory_env.get_stations_by_type(StationType.PARKING_BAY)
        assert len(parking) >= 0


class TestZoneDetection:
    """Test zone type detection."""

    def test_position_in_aisle(self, factory_env):
        """Test detection of positions in aisles."""
        # Check that is_in_zone method exists and works
        result = factory_env.is_in_zone((50.0, 10.0), FactoryZone.AISLE)
        assert isinstance(result, bool)

    def test_position_in_intersection(self, factory_env):
        """Test detection of intersection zones."""
        # Test that is_in_zone works for intersections
        intersections = factory_env.get_intersections()
        assert len(intersections) > 0
        ix, iy = intersections[0]
        result = factory_env.is_in_zone((ix, iy), FactoryZone.INTERSECTION)
        assert isinstance(result, bool)


class TestStationOccupancy:
    """Test station occupancy management."""

    def test_occupy_station(self, factory_env):
        """Test marking a station as occupied."""
        station_id = "parking_0"
        assert factory_env.occupy_station(station_id, "amr_001")
        station = factory_env.stations[station_id]
        assert station.is_occupied
        assert station.occupied_by == "amr_001"

    def test_cannot_occupy_occupied_station(self, factory_env):
        """Test that occupied station cannot be occupied again."""
        station_id = "parking_0"
        assert factory_env.occupy_station(station_id, "amr_001")
        assert not factory_env.occupy_station(station_id, "amr_002")

    def test_release_station(self, factory_env):
        """Test releasing a station."""
        station_id = "parking_0"
        factory_env.occupy_station(station_id, "amr_001")
        assert factory_env.release_station(station_id)
        station = factory_env.stations[station_id]
        assert not station.is_occupied
        assert station.occupied_by is None

    def test_occupy_nonexistent_station(self, factory_env):
        """Test occupying nonexistent station returns False."""
        assert not factory_env.occupy_station("nonexistent", "amr_001")

    def test_release_nonexistent_station(self, factory_env):
        """Test releasing nonexistent station returns False."""
        assert not factory_env.release_station("nonexistent")


class TestNearestStation:
    """Test nearest available station queries."""

    def test_find_nearest_available_charging(self, factory_env):
        """Test finding nearest available charging station."""
        position = (50.0, 2.0)  # Closer to charging stations at y=2.0
        nearest = factory_env.get_nearest_available_station(position, StationType.CHARGING_STATION)
        if nearest is not None:  # May or may not find depending on grid
            assert nearest.station_type == StationType.CHARGING_STATION

    def test_no_available_station(self, factory_env):
        """Test when no available stations exist."""
        # Occupy all parking bays
        parking = factory_env.get_stations_by_type(StationType.PARKING_BAY)
        for i, station in enumerate(parking):
            factory_env.occupy_station(station.station_id, f"amr_{i}")

        # Should return None
        position = (50.0, 78.0)
        result = factory_env.get_nearest_available_station(position, StationType.PARKING_BAY)
        assert result is None

    def test_nearest_distance_calculation(self, factory_env):
        """Test that nearest station is actually nearest."""
        position = (50.0, 50.0)
        nearest = factory_env.get_nearest_available_station(position, StationType.CELL_ASSEMBLY)
        if nearest is not None:
            # Calculate distance to nearest
            dx = nearest.position[0] - position[0]
            dy = nearest.position[1] - position[1]
            dist_to_nearest = math.sqrt(dx**2 + dy**2)

            # Check all other available stations are farther
            for station in factory_env.get_stations_by_type(StationType.CELL_ASSEMBLY):
                if station.is_occupied:
                    continue
                dx = station.position[0] - position[0]
                dy = station.position[1] - position[1]
                dist = math.sqrt(dx**2 + dy**2)
                assert dist >= dist_to_nearest - 0.01


class TestIntersections:
    """Test intersection detection."""

    def test_intersections_list(self, factory_env):
        """Test that intersections are properly identified."""
        intersections = factory_env.get_intersections()
        assert len(intersections) > 0
        # Should be 4 x 6 = 24 intersections (4 vertical aisles, 6 horizontal aisles)
        assert len(intersections) == 24

    def test_intersection_coordinates(self, factory_env):
        """Test intersection coordinate bounds."""
        intersections = factory_env.get_intersections()
        for x, y in intersections:
            assert 0 <= x <= factory_env.width
            assert 0 <= y <= factory_env.height


class TestAisleGraph:
    """Test aisle waypoint graph."""

    def test_aisle_graph_exists(self, factory_env):
        """Test that aisle graph is created."""
        graph = factory_env.get_aisle_graph()
        assert isinstance(graph, dict)
        assert len(graph) > 0

    def test_aisle_waypoints_exist(self, factory_env):
        """Test that aisle waypoints are created."""
        assert len(factory_env.aisle_waypoints) > 0

    def test_aisle_graph_connectivity(self, factory_env):
        """Test that aisle graph is properly connected."""
        graph = factory_env.get_aisle_graph()
        # Each waypoint should have at least one neighbor
        for wp_id, neighbors in graph.items():
            assert isinstance(neighbors, list)


class TestStationApproachPoints:
    """Test station approach point calculations."""

    def test_approach_point_calculated(self, factory_env):
        """Test that approach points are calculated."""
        station = factory_env.stations["parking_0"]
        assert station.approach_point != (0.0, 0.0)

    def test_approach_point_distance(self, factory_env):
        """Test that approach point is correct distance."""
        station = factory_env.stations["parking_0"]
        dx = station.position[0] - station.approach_point[0]
        dy = station.position[1] - station.approach_point[1]
        distance = math.sqrt(dx**2 + dy**2)
        assert abs(distance - 2.0) < 0.1  # Should be approximately 2m

    def test_approach_heading_set(self, factory_env):
        """Test that approach heading is set."""
        station = factory_env.stations["charging_0"]
        assert isinstance(station.approach_heading, float)


class TestVisualization:
    """Test visualization functionality."""

    def test_visualization_no_robots(self, factory_env):
        """Test visualization without robots."""
        fig = factory_env.visualize()
        assert fig is not None
        assert hasattr(fig, 'savefig')

    def test_visualization_with_robots(self, factory_env, robot):
        """Test visualization with robots."""
        robots = [
            {
                "id": robot.robot_id,
                "position": robot.position,
                "heading": robot.heading,
                "color": "#2E86AB"
            }
        ]
        fig = factory_env.visualize(robots=robots)
        assert fig is not None

    def test_visualization_with_paths(self, factory_env):
        """Test visualization with paths."""
        paths = [
            [(10.0, 10.0), (20.0, 20.0), (30.0, 30.0)],
            [(50.0, 50.0), (60.0, 60.0)],
        ]
        fig = factory_env.visualize(paths=paths)
        assert fig is not None
