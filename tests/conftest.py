"""Shared pytest fixtures for all tests."""

import sys
import os
import math
import pytest

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from factory.environment import FactoryEnvironment, StationType
from amr.robot import AMRRobot, AMRSpecs, create_default_fleet
from planning.dubins import DubinsPlanner
from planning.spline import CubicSplineSmoother
from planning.path_planner import FactoryPathPlanner, FactoryEnvironment as PlannerEnv
from docking.dock_controller import DockController
from traffic.traffic_manager import TrafficManager
from fleet.coordinator import FleetCoordinator


@pytest.fixture
def factory_env():
    """Create a factory environment for testing."""
    return FactoryEnvironment(width=100.0, height=80.0, resolution=0.5)


@pytest.fixture
def robot():
    """Create a single test robot."""
    return AMRRobot(
        robot_id="test_amr_001",
        start_position=(50.0, 40.0),
        start_heading=0.0
    )


@pytest.fixture
def fleet(factory_env):
    """Create a fleet of test robots."""
    parking_positions = [
        (15.0, 78.0),
        (30.0, 78.0),
        (45.0, 78.0),
        (60.0, 78.0),
    ]
    return create_default_fleet(4, parking_positions)


@pytest.fixture
def dubins_planner():
    """Create a Dubins path planner."""
    return DubinsPlanner(turning_radius=2.0)


@pytest.fixture
def spline_smoother():
    """Create a cubic spline smoother."""
    return CubicSplineSmoother(min_turning_radius=2.0)


@pytest.fixture
def path_planner(factory_env):
    """Create a factory path planner."""
    planner_env = PlannerEnv(
        aisle_waypoints={
            f"wp_{i}_{j}": (float(x), float(y))
            for i, x in enumerate([12.0, 37.0, 58.0, 77.0])
            for j, y in enumerate([10.0, 20.0, 35.0, 50.0, 65.0, 75.0])
        },
        aisle_connections={
            f"wp_{i}_{j}": [f"wp_{i}_{k}" for k in range(6) if k != j]
            for i in range(4) for j in range(6)
        },
        stations={}
    )
    return FactoryPathPlanner(planner_env, turning_radius=2.0)


@pytest.fixture
def dock_controller():
    """Create a dock controller."""
    return DockController(position_tolerance=0.01, alignment_tolerance=0.02)


@pytest.fixture
def traffic_manager(factory_env):
    """Create a traffic manager."""
    intersections = factory_env.get_intersections()
    return TrafficManager(intersections, safety_radius=1.5)


@pytest.fixture
def fleet_coordinator(factory_env, fleet, path_planner, traffic_manager, dock_controller):
    """Create a fleet coordinator."""
    return FleetCoordinator(
        factory_env=factory_env,
        robots=fleet,
        path_planner=path_planner,
        traffic_manager=traffic_manager,
        dock_controller=dock_controller
    )
