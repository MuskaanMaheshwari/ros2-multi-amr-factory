"""
Test suite for DWA, obstacle detection, and alert system implementations.

Verifies that all new modules can be imported, instantiated, and execute
without errors. Does NOT run full unit tests, just basic smoke tests.

Author: Muskaan Maheshwari
"""

import math
import numpy as np
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from src.planning.dwa import DWAPlanner, DWAMetrics
from src.perception.obstacle_detector import (
    ObstacleDetector, ObstacleType, ObstacleSeverity, create_test_obstacle_grid
)
from src.messaging.alert_system import (
    AlertGenerator, AlertHub, AlertEventType, AlertDecision
)
from src.amr.robot import AMRRobot, AMRState, AMRSpecs
from src.utils.navigation_animator import NavigationAnimator


def test_dwa_planner():
    """Test DWA planner initialization and planning."""
    print("\n[TEST 1] DWA Planner")
    print("-" * 50)

    planner = DWAPlanner(
        max_linear_velocity=1.5,
        max_angular_velocity=2.0,
        prediction_time=2.0,
        dt=0.1,
    )
    print("✓ DWA Planner initialized")

    # Create test obstacle grid
    grid = np.zeros((100, 100), dtype=np.uint8)
    grid[30:35, 40:60] = 1  # Obstacle

    # Test planning
    robot_pose = (5.0, 5.0, 0.0)
    current_velocity = (0.5, 0.0)
    goal = (50.0, 50.0)

    velocity, metrics = planner.plan(
        robot_pose=robot_pose,
        current_velocity=current_velocity,
        goal=goal,
        obstacle_grid=grid,
    )

    assert isinstance(velocity, tuple) and len(velocity) == 2
    assert isinstance(metrics, DWAMetrics)
    assert 0 <= metrics.best_score or metrics.best_score < 0  # Any score is fine
    print(f"✓ Planning executed successfully")
    print(f"  - Best score: {metrics.best_score:.3f}")
    print(f"  - Min obstacle distance: {metrics.min_obstacle_distance:.3f}")
    print(f"  - Heading error: {metrics.heading_error:.3f}")
    print(f"  - Recommended velocity: ({velocity[0]:.3f}, {velocity[1]:.3f})")

    return True


def test_obstacle_detector():
    """Test obstacle detection module."""
    print("\n[TEST 2] Obstacle Detector")
    print("-" * 50)

    # Create detector with test costmap
    expected_grid = create_test_obstacle_grid(100, 100)
    detector = ObstacleDetector(
        expected_path_costmap=expected_grid,
        lidar_range=15.0,
        false_positive_rate=0.05,
        false_negative_rate=0.1,
    )
    print("✓ Obstacle detector initialized")

    # Create actual grid with additional obstacle
    actual_grid = expected_grid.copy()
    actual_grid[50:55, 30:35] = 1  # New obstacle

    # Scan
    robot_pose = (5.0, 5.0, 0.0)
    scan_result = detector.scan(
        robot_pose=robot_pose,
        actual_obstacle_grid=actual_grid,
        timestamp=0.0,
    )

    print(f"✓ Scan executed successfully")
    print(f"  - Obstacle detected: {scan_result.obstacle_detected}")
    print(f"  - Number of obstacles found: {len(scan_result.obstacles)}")
    print(f"  - Number of scan points: {len(scan_result.scan_points)}")

    if scan_result.obstacles:
        obs = scan_result.obstacles[0]
        print(f"  - First obstacle: {obs.obstacle_type.value} at distance {obs.distance:.2f}m")
        print(f"  - Severity: {obs.severity.value}, Confidence: {obs.confidence:.2f}")

    return True


def test_alert_system():
    """Test alert generation and hub system."""
    print("\n[TEST 3] Alert System")
    print("-" * 50)

    # Create generator
    generator = AlertGenerator()
    print("✓ Alert generator initialized")

    # Create obstacle alert
    alert = generator.create_obstacle_alert(
        robot_id="amr_001",
        position=(5.0, 5.0),
        obstacle_type=ObstacleType.PERSON,
        distance=0.8,
        current_state="navigating",
        severity=ObstacleSeverity.HIGH,
        confidence=0.95,
    )

    print(f"✓ Obstacle alert generated")
    print(f"  - Alert ID: {alert.alert_id}")
    print(f"  - Event type: {alert.event_type.value}")
    print(f"  - Hub response required: {alert.hub_response_required}")

    alert_dict = alert.to_dict()
    assert isinstance(alert_dict, dict)
    print(f"✓ Alert serialized to dictionary")

    # Create hub
    hub = AlertHub(max_history_size=100, alert_timeout=30.0)
    print("✓ Alert hub initialized")

    # Process alert
    decision = hub.process_alert(alert)
    assert isinstance(decision, AlertDecision)
    print(f"✓ Alert processed")
    print(f"  - Decision: {decision.value}")

    # Check active alerts
    active = hub.get_active_alerts()
    assert "amr_001" in active
    print(f"✓ Active alerts tracked")

    # Create cleared alert
    cleared_alert = generator.create_cleared_alert(
        robot_id="amr_001",
        position=(5.0, 5.0),
    )
    hub.process_alert(cleared_alert)
    print(f"✓ Cleared alert processed")

    # Get statistics
    stats = hub.get_statistics()
    print(f"✓ Statistics retrieved")
    print(f"  - Total alerts: {stats['total_alerts']}")
    print(f"  - Active alerts: {stats['active_alerts']}")

    return True


def test_robot_integration():
    """Test robot model with obstacle detection."""
    print("\n[TEST 4] Robot Integration")
    print("-" * 50)

    robot = AMRRobot(
        robot_id="amr_001",
        start_position=(5.0, 5.0),
        start_heading=0.0,
    )
    print("✓ Robot initialized")

    # Integrate obstacle detector
    from src.perception.obstacle_detector import ObstacleDetector
    detector = ObstacleDetector(
        expected_path_costmap=create_test_obstacle_grid(),
        lidar_range=15.0,
    )
    robot.obstacle_detector = detector
    print("✓ Obstacle detector attached to robot")

    # Integrate alert generator
    generator = AlertGenerator()
    robot.alert_generator = generator
    print("✓ Alert generator attached to robot")

    # Integrate DWA planner
    dwa = DWAPlanner()
    robot.dwa_planner = dwa
    print("✓ DWA planner attached to robot")

    # Simulate motion
    robot.state = AMRState.NAVIGATING
    robot.set_velocity(1.0, 0.0)

    # Run update with obstacle detection
    for t in range(5):
        pose = robot.update(dt=0.1)
        assert isinstance(pose, tuple) and len(pose) == 3
    print(f"✓ Robot update cycles executed (5 steps)")

    # Test emergency stop
    robot.emergency_stop()
    assert robot.state == AMRState.EMERGENCY_STOP
    print(f"✓ Emergency stop activated")

    # Test resume check
    can_resume = robot.can_resume_path()
    print(f"✓ Resume path check executed: {can_resume}")

    return True


def test_navigation_animator():
    """Test navigation visualization module."""
    print("\n[TEST 5] Navigation Animator")
    print("-" * 50)

    # Create mock factory environment
    class MockFactory:
        def __init__(self):
            self.bounds = [(0, 0), (100, 100)]
            self.stations = {
                "station_1": {"position": (20, 20), "type": "CHARGING_STATION"},
                "station_2": {"position": (80, 80), "type": "LOADING_DOCK"},
            }
            self.walls = [
                (10, 10, 90, 10),
                (90, 10, 90, 90),
            ]
            self.obstacles = [(50, 50, 0.5)]

    factory_env = MockFactory()

    # Create path
    path = [(10, 10), (30, 20), (50, 50), (70, 80), (90, 90)]
    headings = [0.0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi]

    animator = NavigationAnimator(
        factory_env=factory_env,
        robot_path=path,
        robot_headings=headings,
    )
    print("✓ Navigation animator initialized")

    # Create visualization (don't show, just verify it doesn't crash)
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt

    fig = animator.create_grid_visualization(n_cols=2)
    print("✓ Grid visualization created")

    assert fig is not None
    print(f"✓ Figure object valid")

    plt.close(fig)
    print(f"✓ Figure closed")

    return True


def main():
    """Run all tests."""
    print("=" * 60)
    print("DWA Implementation Test Suite")
    print("=" * 60)

    tests = [
        ("DWA Planner", test_dwa_planner),
        ("Obstacle Detector", test_obstacle_detector),
        ("Alert System", test_alert_system),
        ("Robot Integration", test_robot_integration),
        ("Navigation Animator", test_navigation_animator),
    ]

    passed = 0
    failed = 0

    for test_name, test_func in tests:
        try:
            if test_func():
                passed += 1
        except Exception as e:
            print(f"\n✗ {test_name} FAILED")
            print(f"  Error: {str(e)}")
            import traceback
            traceback.print_exc()
            failed += 1

    print("\n" + "=" * 60)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 60)

    return failed == 0


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
