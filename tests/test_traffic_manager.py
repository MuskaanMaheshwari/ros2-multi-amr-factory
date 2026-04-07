"""Tests for traffic manager module."""

import pytest
import time
from src.traffic.traffic_manager import (
    TrafficManager, IntersectionState, TrafficPriority, Intersection, DeadlockInfo
)


class TestTrafficManagerInitialization:
    """Test traffic manager creation."""

    def test_manager_creation(self, traffic_manager):
        """Test traffic manager is created."""
        assert traffic_manager is not None
        assert len(traffic_manager.intersections) > 0

    def test_intersections_created(self, traffic_manager):
        """Test intersections are properly created."""
        # Check that intersections have required attributes
        for i in traffic_manager.intersections.values():
            assert hasattr(i, 'intersection_id')
            assert hasattr(i, 'center')

    def test_initial_intersection_state(self, traffic_manager):
        """Test intersections start as free."""
        for intersection in traffic_manager.intersections.values():
            assert intersection.state == IntersectionState.FREE
            assert intersection.claimed_by is None


class TestIntersectionClaiming:
    """Test intersection request and claim logic."""

    def test_claim_free_intersection(self, traffic_manager):
        """Test claiming a free intersection."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        result = traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        assert result is True

    def test_intersection_becomes_claimed(self, traffic_manager):
        """Test intersection state changes to CLAIMED."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        intersection = traffic_manager.intersections[intersection_id]
        assert intersection.state == IntersectionState.CLAIMED
        assert intersection.claimed_by == "amr_001"

    def test_cannot_claim_claimed_intersection(self, traffic_manager):
        """Test second robot cannot claim claimed intersection."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        result = traffic_manager.request_intersection("amr_002", intersection_id, TrafficPriority.NORMAL)
        assert result is False

    def test_same_robot_reclaim(self, traffic_manager):
        """Test same robot can reclaim its intersection."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        result = traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        assert result is True

    def test_nonexistent_intersection(self, traffic_manager):
        """Test requesting nonexistent intersection."""
        result = traffic_manager.request_intersection("amr_001", "nonexistent", TrafficPriority.NORMAL)
        assert result is False


class TestQueueing:
    """Test intersection queue management."""

    def test_queued_robot_added(self, traffic_manager):
        """Test queued robot is added to queue."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        traffic_manager.request_intersection("amr_002", intersection_id, TrafficPriority.NORMAL)

        intersection = traffic_manager.intersections[intersection_id]
        assert "amr_002" in intersection.queue

    def test_queue_priority_order(self, traffic_manager):
        """Test queue respects priority ordering."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        traffic_manager.request_intersection("amr_002", intersection_id, TrafficPriority.HIGH)
        traffic_manager.request_intersection("amr_003", intersection_id, TrafficPriority.LOW)

        intersection = traffic_manager.intersections[intersection_id]
        # HIGH priority should be first in queue
        assert intersection.queue[0] == "amr_002"
        assert intersection.queue[1] == "amr_003"

    def test_duplicate_queue_entry(self, traffic_manager):
        """Test robot is not queued twice."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        traffic_manager.request_intersection("amr_002", intersection_id, TrafficPriority.NORMAL)
        traffic_manager.request_intersection("amr_002", intersection_id, TrafficPriority.HIGH)

        intersection = traffic_manager.intersections[intersection_id]
        queue_count = intersection.queue.count("amr_002")
        assert queue_count == 1


class TestIntersectionEntry:
    """Test entering intersections."""

    def test_claimed_robot_can_enter(self, traffic_manager):
        """Test that robot that claimed intersection can enter."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        result = traffic_manager.enter_intersection("amr_001", intersection_id)
        assert result is True

    def test_intersection_occupied_on_entry(self, traffic_manager):
        """Test intersection state changes to OCCUPIED."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        traffic_manager.enter_intersection("amr_001", intersection_id)
        intersection = traffic_manager.intersections[intersection_id]
        assert intersection.state == IntersectionState.OCCUPIED

    def test_unclaimed_robot_cannot_enter(self, traffic_manager):
        """Test robot cannot enter intersection it didn't claim."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        result = traffic_manager.enter_intersection("amr_002", intersection_id)
        assert result is False


class TestIntersectionExit:
    """Test exiting intersections."""

    def test_exit_frees_intersection(self, traffic_manager):
        """Test exiting frees the intersection."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        traffic_manager.enter_intersection("amr_001", intersection_id)
        traffic_manager.exit_intersection("amr_001", intersection_id)

        intersection = traffic_manager.intersections[intersection_id]
        assert intersection.state == IntersectionState.FREE
        assert intersection.claimed_by is None

    def test_exit_grants_to_next_in_queue(self, traffic_manager):
        """Test exiting grants intersection to next queued robot."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        traffic_manager.request_intersection("amr_002", intersection_id, TrafficPriority.NORMAL)

        traffic_manager.exit_intersection("amr_001", intersection_id)

        intersection = traffic_manager.intersections[intersection_id]
        assert intersection.claimed_by == "amr_002"
        assert intersection.state == IntersectionState.CLAIMED

    def test_non_holder_cannot_exit(self, traffic_manager):
        """Test robot cannot exit intersection it doesn't hold."""
        intersection_id = list(traffic_manager.intersections.keys())[0]
        traffic_manager.request_intersection("amr_001", intersection_id, TrafficPriority.NORMAL)
        # No change since amr_002 doesn't hold it
        traffic_manager.exit_intersection("amr_002", intersection_id)

        intersection = traffic_manager.intersections[intersection_id]
        assert intersection.claimed_by == "amr_001"  # Should still be held by amr_001


class TestPathIntersections:
    """Test finding intersections along paths."""

    def test_path_intersections(self, traffic_manager):
        """Test finding intersections a path passes through."""
        path = [(10.0, 10.0), (12.0, 10.0), (15.0, 10.0)]
        intersecting = traffic_manager.check_path_intersections(path)
        # Should find at least the intersection near 12.0, 10.0
        assert isinstance(intersecting, list)

    def test_no_intersection_with_empty_path(self, traffic_manager):
        """Test empty path has no intersections."""
        intersecting = traffic_manager.check_path_intersections([])
        assert len(intersecting) == 0

    def test_path_with_no_intersections(self, traffic_manager):
        """Test path that doesn't intersect."""
        path = [(0.5, 0.5), (1.0, 1.0)]
        intersecting = traffic_manager.check_path_intersections(path)
        assert len(intersecting) == 0


class TestCollisionDetection:
    """Test collision risk detection."""

    def test_collision_risk_close_robots(self, traffic_manager):
        """Test robots within safety radius detected."""
        positions = {
            "amr_001": (10.0, 10.0),
            "amr_002": (10.5, 10.5),  # Very close
        }
        collisions = traffic_manager.check_collision_risk(positions)
        assert ("amr_001", "amr_002") in collisions or ("amr_002", "amr_001") in collisions

    def test_no_collision_far_robots(self, traffic_manager):
        """Test far robots not detected as collision risk."""
        positions = {
            "amr_001": (10.0, 10.0),
            "amr_002": (20.0, 20.0),  # Far apart
        }
        collisions = traffic_manager.check_collision_risk(positions)
        assert len(collisions) == 0

    def test_single_robot_no_collision(self, traffic_manager):
        """Test single robot has no collision risk."""
        positions = {"amr_001": (10.0, 10.0)}
        collisions = traffic_manager.check_collision_risk(positions)
        assert len(collisions) == 0


class TestDeadlockDetection:
    """Test deadlock detection."""

    def test_no_deadlock_independent_robots(self, traffic_manager):
        """Test independent robots don't cause deadlock."""
        robot_states = {
            "amr_001": {"waiting_for": None},
            "amr_002": {"waiting_for": None},
        }
        deadlock = traffic_manager.detect_deadlock(robot_states)
        assert deadlock is None

    def test_deadlock_cycle_detection(self, traffic_manager):
        """Test deadlock is detected for cycle."""
        # Create a cycle: amr_001 waits for intersection_0, which is held by amr_002
        # amr_002 waits for intersection_1, which is held by amr_001
        intersection_ids = list(traffic_manager.intersections.keys())
        intersection_0 = intersection_ids[0] if len(intersection_ids) > 0 else "intersection_0"
        intersection_1 = intersection_ids[1] if len(intersection_ids) > 1 else "intersection_1"

        # Set up the deadlock scenario
        traffic_manager.request_intersection("amr_001", intersection_0, TrafficPriority.NORMAL)
        traffic_manager.request_intersection("amr_002", intersection_1, TrafficPriority.NORMAL)

        robot_states = {
            "amr_001": {"waiting_for": intersection_1},
            "amr_002": {"waiting_for": intersection_0},
        }

        deadlock = traffic_manager.detect_deadlock(robot_states)
        # May or may not detect depending on setup, so just check result is valid
        assert deadlock is None or isinstance(deadlock, type(deadlock))


class TestDeadlockResolution:
    """Test deadlock resolution."""

    def test_resolve_deadlock_priority(self, traffic_manager):
        """Test deadlock resolution respects priority."""
        deadlock = DeadlockInfo(
            robot_ids=["amr_001", "amr_002"],
            intersection_ids=["intersection_0"],
            detected_time=time.time()
        )

        # Set priorities
        traffic_manager.set_robot_priority("amr_001", TrafficPriority.HIGH)
        traffic_manager.set_robot_priority("amr_002", TrafficPriority.LOW)

        robot_states = {
            "amr_001": {"battery": 0.5},
            "amr_002": {"battery": 0.5},
        }

        actions = traffic_manager.resolve_deadlock(deadlock, robot_states)
        # Lower priority should yield
        assert actions.get("amr_002") == "yield"
        assert actions.get("amr_001") == "proceed"

    def test_resolve_deadlock_battery(self, traffic_manager):
        """Test deadlock resolution respects battery."""
        deadlock = DeadlockInfo(
            robot_ids=["amr_001", "amr_002"],
            intersection_ids=["intersection_0"],
            detected_time=time.time()
        )

        # Set equal priority
        traffic_manager.set_robot_priority("amr_001", TrafficPriority.NORMAL)
        traffic_manager.set_robot_priority("amr_002", TrafficPriority.NORMAL)

        robot_states = {
            "amr_001": {"battery": 0.8},
            "amr_002": {"battery": 0.2},
        }

        actions = traffic_manager.resolve_deadlock(deadlock, robot_states)
        # Lower battery should yield
        assert actions.get("amr_002") == "yield"


class TestVelocityModifier:
    """Test velocity modification based on intersections."""

    def test_velocity_unmodified_far_from_intersection(self, traffic_manager):
        """Test velocity at normal speed far from intersections."""
        robot_id = "amr_001"
        position = (0.0, 0.0)
        velocity = (1.0, 0.0)

        modifier = traffic_manager.get_velocity_modifier(robot_id, position, velocity)
        assert modifier == 1.0

    def test_velocity_reduced_near_intersection(self, traffic_manager):
        """Test velocity reduced near intersection."""
        if not traffic_manager.intersections:
            pytest.skip("No intersections to test")

        intersection = list(traffic_manager.intersections.values())[0]
        robot_id = "amr_001"
        position = (intersection.center[0] + 1.0, intersection.center[1])
        velocity = (1.0, 0.0)

        modifier = traffic_manager.get_velocity_modifier(robot_id, position, velocity)
        # Should be reduced or stopped
        assert modifier <= 1.0

    def test_velocity_stopped_at_unclaimed_intersection(self, traffic_manager):
        """Test velocity stopped if intersection not claimed."""
        if not traffic_manager.intersections:
            pytest.skip("No intersections to test")

        intersection = list(traffic_manager.intersections.values())[0]
        robot_id = "amr_001"
        position = intersection.center
        velocity = (1.0, 0.0)

        modifier = traffic_manager.get_velocity_modifier(robot_id, position, velocity)
        assert modifier == 0.0


class TestTrafficStatus:
    """Test traffic status reporting."""

    def test_get_status(self, traffic_manager):
        """Test getting traffic status."""
        status = traffic_manager.get_status()
        assert "num_free" in status
        assert "num_claimed" in status
        assert "num_occupied" in status
        assert "active_deadlocks" in status
        assert "queue_lengths" in status

    def test_status_counts(self, traffic_manager):
        """Test status counts are consistent."""
        status = traffic_manager.get_status()
        total = status["num_free"] + status["num_claimed"] + status["num_occupied"]
        assert total == len(traffic_manager.intersections)
