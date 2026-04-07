"""Dubins path planner — optimal curved paths with minimum turning radius.

Implements the six Dubins path types (LSL, LSR, RSL, RSR, RLR, LRL) to find
the shortest path between two poses (x, y, θ) subject to a curvature bound.

The standard approach:
  1. Normalize the problem so the turning radius is 1 and start is at the origin.
  2. Compute all six candidate segment triples (t, p, q) in the normalized frame.
  3. Scale back by the turning radius to get real-world lengths.
  4. Sample the path by stepping through arcs and straight segments.

Reference: Shkel & Lumelsky, "Classification of the Dubins Set" (2001).

Author: Muskaan Maheshwari
"""

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple
import math


TWO_PI = 2.0 * math.pi


class DubinsPathType(Enum):
    LSL = "LSL"
    LSR = "LSR"
    RSL = "RSL"
    RSR = "RSR"
    RLR = "RLR"
    LRL = "LRL"


@dataclass
class DubinsPath:
    path_type: DubinsPathType
    segments: List[Tuple[str, float]]  # [(type, arc_length_meters), ...]
    total_length: float                # meters
    turning_radius: float


def _mod2pi(angle: float) -> float:
    """Map angle into [0, 2π)."""
    return angle - TWO_PI * math.floor(angle / TWO_PI)


class DubinsPlanner:
    """Plan shortest Dubins paths between two poses."""

    def __init__(self, turning_radius: float = 2.0) -> None:
        self.turning_radius = turning_radius

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(
        self,
        start_pose: Tuple[float, float, float],
        goal_pose: Tuple[float, float, float],
    ) -> Optional[DubinsPath]:
        """Find shortest Dubins path.

        Args:
            start_pose: (x, y, θ) in meters / radians.
            goal_pose:  (x, y, θ) in meters / radians.

        Returns:
            DubinsPath or None.
        """
        # Normalize: put start at origin heading 0, distances divided by r
        sx, sy, sa = start_pose
        gx, gy, ga = goal_pose
        r = self.turning_radius

        dx = gx - sx
        dy = gy - sy
        d = math.hypot(dx, dy) / r  # normalized distance
        theta = math.atan2(dy, dx)
        alpha = _mod2pi(sa - theta)  # start heading in local frame
        beta = _mod2pi(ga - theta)   # goal heading in local frame

        # Try all six path types; each returns (t, p, q) or None
        candidates = [
            (DubinsPathType.LSL, self._LSL(alpha, beta, d)),
            (DubinsPathType.RSR, self._RSR(alpha, beta, d)),
            (DubinsPathType.LSR, self._LSR(alpha, beta, d)),
            (DubinsPathType.RSL, self._RSL(alpha, beta, d)),
            (DubinsPathType.RLR, self._RLR(alpha, beta, d)),
            (DubinsPathType.LRL, self._LRL(alpha, beta, d)),
        ]

        best: Optional[DubinsPath] = None
        for ptype, result in candidates:
            if result is None:
                continue
            t, p, q = result
            length = (t + p + q) * r  # scale back to meters
            if best is None or length < best.total_length:
                seg_labels = list(ptype.value)  # e.g. ['L','S','L']
                best = DubinsPath(
                    path_type=ptype,
                    segments=[
                        (seg_labels[0], t * r),
                        (seg_labels[1], p * r),
                        (seg_labels[2], q * r),
                    ],
                    total_length=length,
                    turning_radius=r,
                )
        return best

    def sample_path(
        self,
        dubins_path: DubinsPath,
        start_pose: Tuple[float, float, float],
        step_size: float = 0.1,
    ) -> List[Tuple[float, float, float]]:
        """Dense (x, y, θ) samples along a DubinsPath."""
        r = dubins_path.turning_radius
        x, y, th = start_pose
        points: List[Tuple[float, float, float]] = [(x, y, th)]

        for seg_type, seg_len in dubins_path.segments:
            if seg_len < 1e-9:
                continue
            n = max(1, int(math.ceil(seg_len / step_size)))
            ds = seg_len / n
            for _ in range(n):
                if seg_type == 'S':
                    x += ds * math.cos(th)
                    y += ds * math.sin(th)
                elif seg_type == 'L':
                    dth = ds / r
                    x += r * (math.sin(th + dth) - math.sin(th))
                    y += r * (-math.cos(th + dth) + math.cos(th))
                    th += dth
                elif seg_type == 'R':
                    dth = ds / r
                    x += r * (-math.sin(th - dth) + math.sin(th))
                    y += r * (math.cos(th - dth) - math.cos(th))
                    th -= dth
                points.append((x, y, th))
        return points

    # ------------------------------------------------------------------
    # Six path type solvers (normalized: r = 1)
    # Each returns (t, p, q) with t, q = arc angles, p = straight or arc
    # All values ≥ 0.  Returns None if infeasible.
    # ------------------------------------------------------------------

    @staticmethod
    def _LSL(alpha: float, beta: float, d: float):
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb))
        if tmp < 0:
            return None
        p = math.sqrt(tmp)
        theta = math.atan2(cb - ca, d + sa - sb)
        t = _mod2pi(-alpha + theta)
        q = _mod2pi(beta - theta)
        if t < 0 or p < 0 or q < 0:
            return None
        return (t, p, q)

    @staticmethod
    def _RSR(alpha: float, beta: float, d: float):
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa))
        if tmp < 0:
            return None
        p = math.sqrt(tmp)
        theta = math.atan2(ca - cb, d - sa + sb)
        t = _mod2pi(alpha - theta)
        q = _mod2pi(-beta + theta)
        if t < 0 or p < 0 or q < 0:
            return None
        return (t, p, q)

    @staticmethod
    def _LSR(alpha: float, beta: float, d: float):
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb - d * (sa + sb))
        if tmp < 0:
            return None
        p = math.sqrt(tmp)
        theta = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        t = _mod2pi(-alpha + theta)
        q = _mod2pi(-beta + theta)
        if t < 0 or p < 0 or q < 0:
            return None
        return (t, p, q)

    @staticmethod
    def _RSL(alpha: float, beta: float, d: float):
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb))
        if tmp < 0:
            return None
        p = math.sqrt(tmp)
        theta = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        t = _mod2pi(alpha - theta)
        q = _mod2pi(beta - theta)
        if t < 0 or p < 0 or q < 0:
            return None
        return (t, p, q)

    @staticmethod
    def _RLR(alpha: float, beta: float, d: float):
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        tmp = (6.0 - d * d + 2.0 * (ca * cb + sa * sb) + 2.0 * d * (sa - sb)) / 8.0
        if abs(tmp) > 1.0:
            return None
        p = _mod2pi(2.0 * math.pi - math.acos(tmp))
        t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        q = _mod2pi(alpha - beta - t + p)
        if t < 0 or p < 0 or q < 0:
            return None
        return (t, p, q)

    @staticmethod
    def _LRL(alpha: float, beta: float, d: float):
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        tmp = (6.0 - d * d + 2.0 * (ca * cb + sa * sb) + 2.0 * d * (sb - sa)) / 8.0
        if abs(tmp) > 1.0:
            return None
        p = _mod2pi(2.0 * math.pi - math.acos(tmp))
        t = _mod2pi(-alpha + math.atan2(-ca + cb, d + sa - sb) + p / 2.0)
        q = _mod2pi(beta - alpha - t + p)
        if t < 0 or p < 0 or q < 0:
            return None
        return (t, p, q)
