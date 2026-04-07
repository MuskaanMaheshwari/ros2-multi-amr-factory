"""Tests for AMR robot module."""

import pytest
import math
from src.amr.robot import AMRRobot, AMRSpecs, BatteryModel, AMRState, create_default_fleet


class TestRobotInitialization:
    """Test robot creation and initialization."""

    def test_robot_creation(self, robot):
        """Test basic robot creation."""
        assert robot.robot_id == "test_amr_001"
        assert robot.position == (50.0, 40.0)
        assert robot.heading == 0.0

    def test_robot_default_state(self, robot):
        """Test robot starts in IDLE state."""
        assert robot.state.value == AMRState.IDLE.value

    def test_robot_specs(self, robot):
        """Test robot specs are initialized."""
        assert hasattr(robot.specs, 'length')
        assert robot.specs.length == 1.2
        assert robot.specs.width == 0.8

    def test_robot_battery_initialized(self, robot):
        """Test battery is initialized to full."""
        assert robot.battery.current_charge == 100.0

    def test_robot_trajectory_history(self, robot):
        """Test trajectory history is initialized."""
        assert len(robot.trajectory) == 1
        assert robot.trajectory[0] == (50.0, 40.0)


class TestRobotKinematics:
    """Test robot movement and kinematics."""

    def test_set_velocity(self, robot):
        """Test setting velocities."""
        robot.set_velocity(1.0, 0.5)
        assert robot.linear_velocity == 1.0
        assert robot.angular_velocity == 0.5

    def test_velocity_clamping(self, robot):
        """Test velocities are clamped to max."""
        robot.set_velocity(10.0, 10.0)
        assert robot.linear_velocity <= robot.specs.max_linear_speed
        assert robot.angular_velocity <= robot.specs.max_angular_speed

    def test_update_position(self, robot):
        """Test robot position updates."""
        robot.set_velocity(1.0, 0.0)
        initial_pos = robot.position
        robot.update(dt=1.0)
        new_pos = robot.position
        assert new_pos != initial_pos
        assert new_pos[0] > initial_pos[0]  # Should move east

    def test_update_heading(self, robot):
        """Test robot heading updates."""
        robot.set_velocity(0.0, 1.0)
        initial_heading = robot.heading
        robot.update(dt=1.0)
        new_heading = robot.heading
        assert new_heading != initial_heading

    def test_trajectory_recording(self, robot):
        """Test trajectory history recording."""
        initial_length = len(robot.trajectory)
        robot.set_velocity(1.0, 0.0)
        robot.update(dt=0.1)
        assert len(robot.trajectory) > initial_length

    def test_straight_line_motion(self, robot):
        """Test straight line motion."""
        robot.set_velocity(1.0, 0.0)
        robot.update(dt=1.0)
        x, y = robot.position
        assert abs(y - 40.0) < 0.01  # Y should not change much
        assert x > 50.0  # X should increase

    def test_in_place_rotation(self, robot):
        """Test 360-degree in-place rotation."""
        robot.set_velocity(0.0, math.pi)  # Rotate at pi rad/s
        initial_pos = robot.position
        robot.update(dt=1.0)
        new_pos = robot.position
        assert abs(initial_pos[0] - new_pos[0]) < 0.01
        assert abs(initial_pos[1] - new_pos[1]) < 0.01


class TestTurretRotation:
    """Test independent turret rotation."""

    def test_rotate_turret(self, robot):
        """Test turret rotation."""
        target = math.pi / 2
        robot.rotate_turret(target, dt=1.0)
        assert robot.turret_angular_velocity != 0.0

    def test_turret_reaches_target(self, robot):
        """Test turret reaches target heading."""
        target = 0.0
        for _ in range(100):
            is_aligned = robot.rotate_turret(target, dt=0.1)
            robot.update(dt=0.1)
            if is_aligned:
                break
        assert abs(robot.turret_heading - target) < 0.01

    def test_turret_independent_from_base(self, robot):
        """Test turret rotates independently from base."""
        robot.set_velocity(1.0, 0.0)
        robot.turret_angular_velocity = 2.0
        robot.update(dt=0.5)
        assert robot.heading != robot.turret_heading


class TestBatteryManagement:
    """Test battery discharge and charging."""

    def test_battery_discharge_on_movement(self, robot):
        """Test battery discharges during movement."""
        initial_charge = robot.battery.current_charge
        robot.set_velocity(1.0, 0.0)
        robot.update(dt=1.0)
        assert robot.battery.current_charge < initial_charge

    def test_battery_discharge_distance_based(self, robot):
        """Test battery discharge is distance-based."""
        distance = 10.0
        robot.battery.discharge(distance)
        expected_discharge = distance * robot.battery.discharge_rate_per_meter
        assert robot.battery.current_charge == 100.0 - expected_discharge

    def test_battery_discharge_idle(self, robot):
        """Test battery discharges while idle."""
        initial_charge = robot.battery.current_charge
        robot.battery.discharge_idle(dt=10.0)
        assert robot.battery.current_charge < initial_charge

    def test_battery_charging(self, robot):
        """Test battery charges."""
        robot.battery.current_charge = 50.0
        initial_charge = robot.battery.current_charge
        robot.battery.charge(dt=1.0)
        assert robot.battery.current_charge > initial_charge

    def test_battery_clamping(self, robot):
        """Test battery is clamped to 0-100%."""
        robot.battery.discharge(1000.0)  # Force discharge
        assert robot.battery.current_charge >= 0.0
        robot.battery.current_charge = 100.0
        robot.battery.charge(1000.0)  # Force charge
        assert robot.battery.current_charge <= 100.0

    def test_battery_low_threshold(self, robot):
        """Test battery low warning."""
        robot.battery.current_charge = 15.0
        assert robot.battery.is_low()
        robot.battery.current_charge = 25.0
        assert not robot.battery.is_low()

    def test_battery_critical_threshold(self, robot):
        """Test battery critical warning."""
        robot.battery.current_charge = 5.0
        assert robot.battery.is_critical()
        robot.battery.current_charge = 15.0
        assert not robot.battery.is_critical()

    def test_battery_can_reach(self, robot):
        """Test battery range estimation."""
        robot.battery.current_charge = 100.0
        assert robot.battery.can_reach(10.0)
        robot.battery.current_charge = 5.0
        assert not robot.battery.can_reach(10.0)

    def test_battery_energy_wh(self, robot):
        """Test energy calculation in watt-hours."""
        robot.battery.current_charge = 100.0
        energy = robot.battery.energy_wh()
        expected = (100.0 / 100.0) * 100.0 * 48.0
        assert energy == expected


class TestPayloadHandling:
    """Test payload management."""

    def test_load_payload(self, robot):
        """Test loading payload."""
        robot.load_payload(100.0)
        assert robot.payload_kg == 100.0

    def test_load_payload_exceeds_capacity(self, robot):
        """Test payload capacity constraint."""
        with pytest.raises(ValueError):
            robot.load_payload(600.0)  # Max is 500kg

    def test_unload_payload(self, robot):
        """Test unloading payload."""
        robot.load_payload(100.0)
        unloaded = robot.unload_payload()
        assert unloaded == 100.0
        assert robot.payload_kg == 0.0

    def test_max_payload(self, robot):
        """Test maximum payload."""
        robot.load_payload(robot.specs.max_payload_kg)
        assert robot.payload_kg == robot.specs.max_payload_kg


class TestStateTransitions:
    """Test state machine transitions."""

    def test_state_idle_to_navigating(self, robot):
        """Test transition to navigating."""
        robot.state = AMRState.NAVIGATING
        assert robot.state == AMRState.NAVIGATING

    def test_start_charging(self, robot):
        """Test charging state."""
        robot.start_charging()
        assert robot.state.value == AMRState.CHARGING.value
        assert robot.linear_velocity == 0.0

    def test_stop_charging(self, robot):
        """Test stop charging."""
        robot.start_charging()
        robot.stop_charging()
        assert robot.state.value == AMRState.IDLE.value

    def test_emergency_stop(self, robot):
        """Test emergency stop."""
        robot.set_velocity(1.0, 1.0)
        robot.emergency_stop()
        assert robot.state.value == AMRState.EMERGENCY_STOP.value
        assert robot.linear_velocity == 0.0
        assert robot.angular_velocity == 0.0

    def test_emergency_stop_halts_motion(self, robot):
        """Test that emergency stop prevents motion."""
        robot.set_velocity(1.0, 0.0)
        robot.emergency_stop()
        initial_pos = robot.position
        robot.update(dt=1.0)
        assert robot.position == initial_pos


class TestCollisionDetection:
    """Test collision detection."""

    def test_footprint_corners(self, robot):
        """Test robot footprint calculation."""
        footprint = robot.get_footprint()
        assert len(footprint) == 4
        for x, y in footprint:
            assert isinstance(x, float)
            assert isinstance(y, float)

    def test_footprint_rotation(self, robot):
        """Test footprint rotates with heading."""
        robot.heading = 0.0
        footprint_0 = robot.get_footprint()
        robot.heading = math.pi / 2
        footprint_90 = robot.get_footprint()
        assert footprint_0 != footprint_90

    def test_safety_circle(self, robot):
        """Test safety circle for collision avoidance."""
        center, radius = robot.get_safety_circle()
        assert center == robot.position
        assert radius == robot.specs.safety_radius

    def test_distance_to_target(self, robot):
        """Test distance calculation."""
        target = (60.0, 50.0)
        dist = robot.distance_to(target)
        expected = math.sqrt((60.0-50.0)**2 + (50.0-40.0)**2)
        assert abs(dist - expected) < 0.01


class TestStatusSerialization:
    """Test robot status serialization."""

    def test_get_status_dict(self, robot):
        """Test robot status dictionary."""
        status = robot.get_status_dict()
        assert "robot_id" in status
        assert "position" in status
        assert "heading_rad" in status
        assert "state" in status
        assert "battery" in status

    def test_status_battery_info(self, robot):
        """Test battery info in status."""
        status = robot.get_status_dict()
        battery = status["battery"]
        assert "charge_percent" in battery
        assert "energy_wh" in battery
        assert "is_low" in battery
        assert "is_critical" in battery


class TestFleetCreation:
    """Test fleet creation."""

    def test_create_fleet(self, fleet):
        """Test fleet is created."""
        assert len(fleet) == 4
        assert "amr_001" in fleet
        assert "amr_004" in fleet

    def test_fleet_robot_ids(self, fleet):
        """Test fleet robot ID format."""
        for robot_id in fleet.keys():
            assert robot_id.startswith("amr_")

    def test_fleet_positions_unique(self, fleet):
        """Test each robot has unique parking position."""
        positions = [robot.position for robot in fleet.values()]
        assert len(positions) == len(set(positions))

    def test_fleet_initial_headings(self, fleet):
        """Test robots have staggered initial headings."""
        headings = [robot.heading for robot in fleet.values()]
        # First 4 should be 0, pi/2, pi, 3pi/2
        expected = [0.0, math.pi/2, math.pi, 3*math.pi/2]
        for actual, exp in zip(headings, expected):
            assert abs(actual - exp) < 0.01

    def test_fleet_creation_insufficient_parking(self):
        """Test fleet creation fails with insufficient parking."""
        with pytest.raises(ValueError):
            create_default_fleet(5, [(0, 0), (1, 1)])


class TestAngleNormalization:
    """Test angle normalization utilities."""

    def test_normalize_positive_angle(self, robot):
        """Test normalization of positive angle."""
        angle = 2 * math.pi + 0.5
        normalized = robot._normalize_angle(angle)
        assert -math.pi <= normalized <= math.pi

    def test_normalize_negative_angle(self, robot):
        """Test normalization of negative angle."""
        angle = -2 * math.pi - 0.5
        normalized = robot._normalize_angle(angle)
        assert -math.pi <= normalized <= math.pi

    def test_angle_difference(self, robot):
        """Test angle difference calculation."""
        target = math.pi / 2
        current = -math.pi / 2
        diff = robot._angle_difference(target, current)
        assert -math.pi <= diff <= math.pi
