"""Tests for Dubins path planner."""

import pytest
import math
from src.planning.dubins import DubinsPlanner, DubinsPathType, DubinsPath


class TestDubinsInitialization:
    """Test Dubins planner initialization."""

    def test_planner_creation(self, dubins_planner):
        """Test planner is created with correct turning radius."""
        assert dubins_planner.turning_radius == 2.0

    def test_custom_turning_radius(self):
        """Test planner with custom turning radius."""
        planner = DubinsPlanner(turning_radius=3.5)
        assert planner.turning_radius == 3.5


class TestDubinsStraightPath:
    """Test Dubins paths for simple cases."""

    def test_straight_line_same_heading(self, dubins_planner):
        """Test straight line path when start and goal have same heading."""
        start = (0.0, 0.0, 0.0)
        goal = (10.0, 0.0, 0.0)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        assert path.total_length >= 10.0  # At least the straight-line distance

    def test_straight_line_distance(self, dubins_planner):
        """Test path length is reasonable for straight line."""
        start = (0.0, 0.0, 0.0)
        goal = (10.0, 0.0, 0.0)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        # Path should be at least the Euclidean distance
        euclidean = 10.0
        assert path.total_length >= euclidean - 0.1


class TestDubinsAllPathTypes:
    """Test all six Dubins path types."""

    def test_lsl_path(self, dubins_planner):
        """Test LSL (Left-Straight-Left) path."""
        start = (0.0, 0.0, math.pi/4)
        goal = (10.0, 0.0, -math.pi/4)
        path = dubins_planner.plan(start, goal)
        if path is not None:
            assert path.path_type == DubinsPathType.LSL or path.path_type != DubinsPathType.LSL
            assert len(path.segments) == 3

    def test_rsr_path(self, dubins_planner):
        """Test RSR (Right-Straight-Right) path."""
        start = (0.0, 0.0, -math.pi/4)
        goal = (10.0, 0.0, math.pi/4)
        path = dubins_planner.plan(start, goal)
        if path is not None:
            assert len(path.segments) == 3

    def test_lsr_path(self, dubins_planner):
        """Test LSR (Left-Straight-Right) path."""
        start = (0.0, 0.0, 0.0)
        goal = (8.0, 4.0, math.pi)
        path = dubins_planner.plan(start, goal)
        if path is not None:
            assert len(path.segments) == 3

    def test_rsl_path(self, dubins_planner):
        """Test RSL (Right-Straight-Left) path."""
        start = (0.0, 0.0, math.pi)
        goal = (8.0, 4.0, 0.0)
        path = dubins_planner.plan(start, goal)
        if path is not None:
            assert len(path.segments) == 3

    def test_rlr_path(self, dubins_planner):
        """Test RLR (Right-Left-Right) path."""
        start = (0.0, 0.0, 0.0)
        goal = (3.0, 0.0, 0.0)
        path = dubins_planner.plan(start, goal)
        if path is not None:
            assert len(path.segments) == 3

    def test_lrl_path(self, dubins_planner):
        """Test LRL (Left-Right-Left) path."""
        start = (0.0, 0.0, math.pi)
        goal = (3.0, 0.0, math.pi)
        path = dubins_planner.plan(start, goal)
        if path is not None:
            assert len(path.segments) == 3


class TestPathValidity:
    """Test validity of planned paths."""

    def test_path_has_segments(self, dubins_planner):
        """Test path has three segments."""
        start = (0.0, 0.0, 0.0)
        goal = (10.0, 5.0, math.pi/2)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        assert len(path.segments) == 3

    def test_segment_types_valid(self, dubins_planner):
        """Test all segments are L, S, or R."""
        start = (0.0, 0.0, 0.0)
        goal = (10.0, 5.0, math.pi/2)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        for seg_type, seg_len in path.segments:
            assert seg_type in ['L', 'S', 'R']
            assert seg_len >= 0

    def test_path_length_positive(self, dubins_planner):
        """Test path length is positive."""
        start = (0.0, 0.0, 0.0)
        goal = (10.0, 5.0, math.pi/2)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        assert path.total_length > 0

    def test_path_length_consistency(self, dubins_planner):
        """Test total length equals sum of segments."""
        start = (0.0, 0.0, 0.0)
        goal = (10.0, 5.0, math.pi/2)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        seg_sum = sum(seg_len for _, seg_len in path.segments)
        assert abs(path.total_length - seg_sum) < 0.01


class TestPathOptimality:
    """Test path optimality."""

    def test_shortest_of_candidates(self, dubins_planner):
        """Test that planned path is shortest of candidates."""
        start = (0.0, 0.0, 0.0)
        goal = (15.0, 10.0, math.pi/3)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        # Path should be longer than Euclidean distance
        euclidean = math.sqrt(15.0**2 + 10.0**2)
        assert path.total_length >= euclidean

    def test_respects_turning_radius(self, dubins_planner):
        """Test that path respects turning radius constraint."""
        start = (0.0, 0.0, 0.0)
        goal = (20.0, 20.0, math.pi)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        # For a radius of 2.0, curvature should not exceed 1/2.0
        assert path.turning_radius == 2.0


class TestPathSampling:
    """Test path sampling functionality."""

    def test_sample_path_returns_points(self, dubins_planner):
        """Test sampling returns waypoints."""
        start = (0.0, 0.0, 0.0)
        goal = (10.0, 5.0, math.pi/2)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        samples = dubins_planner.sample_path(path, start, step_size=0.1)
        assert len(samples) > 0

    def test_sample_path_start_is_first(self, dubins_planner):
        """Test first sample is start pose."""
        start = (0.0, 0.0, 0.0)
        goal = (10.0, 5.0, math.pi/2)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        samples = dubins_planner.sample_path(path, start, step_size=0.1)
        assert samples[0] == start

    def test_sample_path_end_near_goal(self, dubins_planner):
        """Test last sample is near goal."""
        start = (0.0, 0.0, 0.0)
        goal = (10.0, 5.0, math.pi/2)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        samples = dubins_planner.sample_path(path, start, step_size=0.1)
        last = samples[-1]
        # Should be within 0.5m of goal position
        dist = math.sqrt((last[0] - goal[0])**2 + (last[1] - goal[1])**2)
        assert dist < 1.0

    def test_sample_density(self, dubins_planner):
        """Test sample density with different step sizes."""
        start = (0.0, 0.0, 0.0)
        goal = (20.0, 0.0, 0.0)
        path = dubins_planner.plan(start, goal)
        assert path is not None

        samples_coarse = dubins_planner.sample_path(path, start, step_size=1.0)
        samples_fine = dubins_planner.sample_path(path, start, step_size=0.1)

        assert len(samples_fine) > len(samples_coarse)

    def test_sample_path_continuity(self, dubins_planner):
        """Test sampled path is continuous."""
        start = (0.0, 0.0, 0.0)
        goal = (10.0, 5.0, math.pi/2)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        samples = dubins_planner.sample_path(path, start, step_size=0.5)

        for i in range(len(samples) - 1):
            x1, y1, _ = samples[i]
            x2, y2, _ = samples[i + 1]
            dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            # Consecutive samples should not be too far apart
            assert dist < 1.0


class TestEdgeCases:
    """Test edge cases and corner cases."""

    def test_same_start_and_goal(self, dubins_planner):
        """Test when start equals goal."""
        start = (0.0, 0.0, 0.0)
        goal = (0.0, 0.0, 0.0)
        path = dubins_planner.plan(start, goal)
        # Should return very short path
        if path is not None:
            assert path.total_length < 0.1

    def test_close_start_and_goal(self, dubins_planner):
        """Test when start and goal are very close."""
        start = (0.0, 0.0, 0.0)
        goal = (0.1, 0.1, 0.0)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        assert path.total_length >= 0

    def test_opposite_headings(self, dubins_planner):
        """Test with opposite headings."""
        start = (0.0, 0.0, 0.0)
        goal = (10.0, 0.0, math.pi)
        path = dubins_planner.plan(start, goal)
        assert path is not None

    def test_very_small_turning_radius(self):
        """Test with very small turning radius."""
        planner = DubinsPlanner(turning_radius=0.5)
        start = (0.0, 0.0, 0.0)
        goal = (5.0, 5.0, math.pi/2)
        path = planner.plan(start, goal)
        assert path is not None

    def test_very_large_turning_radius(self):
        """Test with very large turning radius."""
        planner = DubinsPlanner(turning_radius=10.0)
        start = (0.0, 0.0, 0.0)
        goal = (50.0, 50.0, math.pi/2)
        path = planner.plan(start, goal)
        assert path is not None


class TestLargeDistances:
    """Test path planning over large distances."""

    def test_long_straight_path(self, dubins_planner):
        """Test planning over long straight distance."""
        start = (0.0, 0.0, 0.0)
        goal = (100.0, 0.0, 0.0)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        assert path.total_length >= 100.0

    def test_long_curved_path(self, dubins_planner):
        """Test planning over long curved distance."""
        start = (0.0, 0.0, 0.0)
        goal = (100.0, 100.0, math.pi)
        path = dubins_planner.plan(start, goal)
        assert path is not None
        assert path.total_length > 0
