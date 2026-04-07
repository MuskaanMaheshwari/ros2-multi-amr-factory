"""Cubic spline path smoother — produces smooth, curvature-continuous paths.

Fits parametric cubic splines through waypoints and enforces turning radius constraints.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple
import math
import numpy as np
from scipy.interpolate import CubicSpline


@dataclass
class SplinePath:
    """Representation of a smoothed spline path."""
    waypoints: List[Tuple[float, float]]  # Original waypoints
    smooth_points: List[Tuple[float, float, float]]  # (x, y, heading) at each sample
    total_length: float
    max_curvature: float


class CubicSplineSmoother:
    """Smooth a sequence of waypoints with cubic splines."""

    def __init__(self, min_turning_radius: float = 2.0) -> None:
        """Initialize the cubic spline smoother.

        Args:
            min_turning_radius: Minimum turning radius in meters (default 2.0m).
        """
        self.min_turning_radius = min_turning_radius
        self.max_curvature = 1.0 / min_turning_radius
        self._EPSILON = 1e-6

    def smooth(
        self,
        waypoints: List[Tuple[float, float]],
        num_samples: int = 200,
    ) -> SplinePath:
        """Fit and sample cubic splines through waypoints.

        Args:
            waypoints: List of (x, y) waypoints to smooth.
            num_samples: Number of output samples.

        Returns:
            SplinePath with smooth points and curvature info.
        """
        if len(waypoints) < 2:
            raise ValueError("At least 2 waypoints required")

        waypoints_array = np.array(waypoints)
        n_points = len(waypoints)

        # Parametrize by index (0, 1, 2, ..., n-1)
        t_orig = np.arange(n_points, dtype=float)

        # Fit cubic splines for x(t) and y(t)
        cs_x = CubicSpline(t_orig, waypoints_array[:, 0], bc_type="natural")
        cs_y = CubicSpline(t_orig, waypoints_array[:, 1], bc_type="natural")

        # Sample at uniform intervals in parameter space
        t_samples = np.linspace(0, n_points - 1, num_samples)

        # Evaluate splines and derivatives
        x_samples = cs_x(t_samples)
        y_samples = cs_y(t_samples)
        dx_samples = cs_x(t_samples, 1)  # First derivative
        dy_samples = cs_y(t_samples, 1)
        ddx_samples = cs_x(t_samples, 2)  # Second derivative
        ddy_samples = cs_y(t_samples, 2)

        # Compute headings, arc length, and curvatures
        smooth_points: List[Tuple[float, float, float]] = []
        curvatures: List[float] = []
        arc_lengths: List[float] = [0.0]

        for i in range(num_samples):
            x = x_samples[i]
            y = y_samples[i]
            dx = dx_samples[i]
            dy = dy_samples[i]
            ddx = ddx_samples[i]
            ddy = ddy_samples[i]

            # Heading from tangent
            theta = math.atan2(dy, dx)
            smooth_points.append((x, y, theta))

            # Curvature: |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
            denom = dx**2 + dy**2
            if denom > self._EPSILON:
                curvature = abs(dx * ddy - dy * ddx) / (denom**1.5)
                curvatures.append(curvature)
            else:
                curvatures.append(0.0)

            # Arc length (approximate via Euclidean)
            if i > 0:
                dx_seg = x - smooth_points[i - 1][0]
                dy_seg = y - smooth_points[i - 1][1]
                arc_lengths.append(arc_lengths[-1] + math.sqrt(dx_seg**2 + dy_seg**2))

        total_length = arc_lengths[-1]
        max_curvature = max(curvatures) if curvatures else 0.0

        # Check if curvature constraint is violated
        if max_curvature > self.max_curvature + self._EPSILON:
            # Add intermediate waypoints and re-smooth
            return self._smooth_with_refinement(
                waypoints, max_curvature, num_samples
            )

        return SplinePath(
            waypoints=waypoints,
            smooth_points=smooth_points,
            total_length=total_length,
            max_curvature=max_curvature,
        )

    def smooth_with_headings(
        self,
        waypoints_with_headings: List[Tuple[float, float, float]],
        num_samples: int = 200,
    ) -> SplinePath:
        """Smooth waypoints while respecting heading constraints.

        Args:
            waypoints_with_headings: List of (x, y, theta) with desired headings.
            num_samples: Number of output samples.

        Returns:
            SplinePath respecting heading constraints at endpoints.
        """
        if len(waypoints_with_headings) < 2:
            raise ValueError("At least 2 waypoints required")

        waypoints_array = np.array(
            [(w[0], w[1]) for w in waypoints_with_headings]
        )
        headings = np.array([w[2] for w in waypoints_with_headings])

        n_points = len(waypoints_array)
        t_orig = np.arange(n_points, dtype=float)

        # Use clamped boundary conditions with heading constraints
        # Boundary slope = (dx/dt, dy/dt) = (cos(theta), sin(theta)) at start/end
        start_heading = headings[0]
        end_heading = headings[-1]

        bc_start = (1, (math.cos(start_heading), math.sin(start_heading)))
        bc_end = (1, (math.cos(end_heading), math.sin(end_heading)))

        cs_x = CubicSpline(
            t_orig, waypoints_array[:, 0], bc_type=("clamped", bc_start[1][0])
        )
        cs_y = CubicSpline(
            t_orig, waypoints_array[:, 1], bc_type=("clamped", bc_start[1][1])
        )

        # Sample and compute like smooth()
        t_samples = np.linspace(0, n_points - 1, num_samples)

        x_samples = cs_x(t_samples)
        y_samples = cs_y(t_samples)
        dx_samples = cs_x(t_samples, 1)
        dy_samples = cs_y(t_samples, 1)
        ddx_samples = cs_x(t_samples, 2)
        ddy_samples = cs_y(t_samples, 2)

        smooth_points: List[Tuple[float, float, float]] = []
        curvatures: List[float] = []
        arc_lengths: List[float] = [0.0]

        for i in range(num_samples):
            x = x_samples[i]
            y = y_samples[i]
            dx = dx_samples[i]
            dy = dy_samples[i]
            ddx = ddx_samples[i]
            ddy = ddy_samples[i]

            theta = math.atan2(dy, dx)
            smooth_points.append((x, y, theta))

            denom = dx**2 + dy**2
            if denom > self._EPSILON:
                curvature = abs(dx * ddy - dy * ddx) / (denom**1.5)
                curvatures.append(curvature)
            else:
                curvatures.append(0.0)

            if i > 0:
                dx_seg = x - smooth_points[i - 1][0]
                dy_seg = y - smooth_points[i - 1][1]
                arc_lengths.append(arc_lengths[-1] + math.sqrt(dx_seg**2 + dy_seg**2))

        total_length = arc_lengths[-1]
        max_curvature = max(curvatures) if curvatures else 0.0

        if max_curvature > self.max_curvature + self._EPSILON:
            waypoints_2d = [
                (w[0], w[1]) for w in waypoints_with_headings
            ]
            return self._smooth_with_refinement(
                waypoints_2d, max_curvature, num_samples
            )

        return SplinePath(
            waypoints=[(w[0], w[1]) for w in waypoints_with_headings],
            smooth_points=smooth_points,
            total_length=total_length,
            max_curvature=max_curvature,
        )

    def _smooth_with_refinement(
        self,
        waypoints: List[Tuple[float, float]],
        max_curvature: float,
        num_samples: int,
    ) -> SplinePath:
        """Iteratively refine waypoints when curvature constraint is violated.

        Args:
            waypoints: Original waypoints.
            max_curvature: Current maximum curvature.
            num_samples: Number of output samples.

        Returns:
            SplinePath with refined waypoints satisfying curvature constraint.
        """
        refined = list(waypoints)
        iterations = 0
        max_iterations = 5

        while (
            max_curvature > self.max_curvature + self._EPSILON
            and iterations < max_iterations
        ):
            iterations += 1

            # Add midpoints between consecutive waypoints
            new_refined: List[Tuple[float, float]] = []
            for i in range(len(refined) - 1):
                new_refined.append(refined[i])
                mx = (refined[i][0] + refined[i + 1][0]) / 2
                my = (refined[i][1] + refined[i + 1][1]) / 2
                new_refined.append((mx, my))
            new_refined.append(refined[-1])

            refined = new_refined

            # Re-smooth with refined waypoints
            spline_path = self.smooth(refined, num_samples)
            max_curvature = spline_path.max_curvature

        return SplinePath(
            waypoints=refined,
            smooth_points=spline_path.smooth_points,
            total_length=spline_path.total_length,
            max_curvature=spline_path.max_curvature,
        )
