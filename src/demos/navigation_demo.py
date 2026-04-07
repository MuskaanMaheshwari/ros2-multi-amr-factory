"""
Multi-Frame Navigation Visualization Demo

Demonstrates Dubins curve path planning with 6 snapshots of robot movement.
Shows the complete path from start to goal with approach point and station,
visualized as a 2x3 grid showing robot progression along the curve.

The Dubins curve is an optimal path connecting two poses (position + orientation)
using circular arcs and straight lines. This is ideal for robots with curvature constraints.

Author: Muskaan Maheshwari
"""

import sys
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Headless rendering
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Circle, Rectangle
from matplotlib.transforms import Affine2D

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from factory.environment import FactoryEnvironment
from planning.dubins import DubinsPlanner


def draw_robot(ax, x, y, heading, size=1.5, color='#2E86AB', alpha=1.0):
    """
    Draw a robot at position with heading arrow.

    Args:
        ax: Matplotlib axes
        x, y: Robot position
        heading: Robot heading in radians
        size: Robot size
        color: Robot color
        alpha: Transparency
    """
    # Robot body (rectangle)
    robot_rect = Rectangle(
        (x - size/2, y - size/2), size, size,
        angle=math.degrees(heading),
        facecolor=color, edgecolor='black', linewidth=2, alpha=alpha
    )
    ax.add_patch(robot_rect)

    # Heading arrow
    arrow_length = size * 1.2
    dx = arrow_length * math.cos(heading)
    dy = arrow_length * math.sin(heading)

    arrow = FancyArrowPatch(
        (x, y), (x + dx, y + dy),
        arrowstyle='->', mutation_scale=20,
        color=color, linewidth=2.5, alpha=alpha
    )
    ax.add_patch(arrow)


def draw_station(ax, x, y, marker_size=8, color='red'):
    """
    Draw a docking station as a circle.

    Args:
        ax: Matplotlib axes
        x, y: Station position
        marker_size: Size of marker
        color: Marker color
    """
    circle = Circle((x, y), marker_size/40, color=color, zorder=5, alpha=0.8)
    ax.add_patch(circle)
    ax.plot(x, y, 'x', color=color, markersize=10, markeredgewidth=2, zorder=6)


def create_navigation_demo(output_path):
    """
    Create a multi-frame navigation visualization showing Dubins path.

    Generates a 2x3 grid showing robot movement at 0%, 20%, 40%, 60%, 80%, 100%
    along a Dubins curve from start to goal pose.

    Args:
        output_path: Path to save the PNG image
    """
    print("Creating navigation demo...")

    # Create factory environment
    factory = FactoryEnvironment(width=100.0, height=80.0, resolution=0.5)

    # Initialize Dubins planner
    dubins_planner = DubinsPlanner(turning_radius=3.0)

    # Define start and goal poses (x, y, heading)
    start_pose = (15.0, 20.0, 0.0)
    goal_pose = (75.0, 60.0, math.pi / 4)

    # Plan path
    dubins_path = dubins_planner.plan(start_pose, goal_pose)

    if not dubins_path:
        print("Error: Could not plan Dubins path")
        return

    # Sample waypoints along the path
    waypoints = dubins_planner.sample_path(dubins_path, start_pose, step_size=0.1)

    # Extract x, y, heading from waypoints
    positions = np.array([(w[0], w[1]) for w in waypoints])
    headings = np.array([w[2] for w in waypoints])

    print(f"Dubins path planned: {len(waypoints)} waypoints")
    print(f"Path type: {dubins_path.path_type.value}")
    print(f"Total length: {dubins_path.total_length:.2f}m")

    # Create approach point (1.5m before goal)
    goal_approach_dist = 1.5
    approach_x = goal_pose[0] - goal_approach_dist * math.cos(goal_pose[2])
    approach_y = goal_pose[1] - goal_approach_dist * math.sin(goal_pose[2])

    # Create 2x3 grid of snapshots
    fig, axes = plt.subplots(2, 3, figsize=(14, 10))
    fig.suptitle(
        'Dubins Curve Path Planning - Multi-Frame Navigation\n'
        'Optimal path connecting two poses with curvature constraints',
        fontsize=14, fontweight='bold', y=0.98
    )

    # Snapshot percentages
    percentages = [0, 20, 40, 60, 80, 100]

    for idx, (ax, percentage) in enumerate(zip(axes.flat, percentages)):
        # Set up axes
        ax.set_xlim(-5, 85)
        ax.set_ylim(-5, 75)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_xlabel('X (m)', fontsize=10)
        ax.set_ylabel('Y (m)', fontsize=10)
        ax.set_title(f'Progress: {percentage}%', fontsize=11, fontweight='bold')

        # Draw factory background (simple grid)
        for i in range(0, 90, 10):
            ax.axvline(i, color='lightgray', linewidth=0.5, alpha=0.5)
            ax.axhline(i, color='lightgray', linewidth=0.5, alpha=0.5)

        # Draw complete path (faint)
        ax.plot(positions[:, 0], positions[:, 1], 'gray', linewidth=1.5,
                alpha=0.4, label='Planned path', zorder=1)

        # Draw start position
        ax.plot(start_pose[0], start_pose[1], 'go', markersize=10,
                label='Start', zorder=4, markeredgecolor='black', markeredgewidth=1.5)

        # Draw approach point
        ax.plot(approach_x, approach_y, 'bs', markersize=8,
                label='Approach point', zorder=4, markeredgecolor='black', markeredgewidth=1)

        # Draw goal/station
        draw_station(ax, goal_pose[0], goal_pose[1], marker_size=10, color='red')
        ax.text(goal_pose[0] + 2, goal_pose[1] + 2, 'Station',
                fontsize=9, color='red', fontweight='bold')

        # Calculate current waypoint index
        current_idx = int(len(waypoints) * percentage / 100)
        current_idx = min(current_idx, len(waypoints) - 1)

        # Draw traveled path (solid)
        if current_idx > 0:
            ax.plot(positions[:current_idx, 0], positions[:current_idx, 1],
                   'b-', linewidth=2.5, alpha=0.8, label='Traveled', zorder=2)

        # Draw remaining path (dashed)
        if current_idx < len(waypoints) - 1:
            ax.plot(positions[current_idx:, 0], positions[current_idx:, 1],
                   'gray', linewidth=1.5, linestyle='--', alpha=0.5, zorder=1)

        # Draw current robot position and heading
        current_x = positions[current_idx, 0]
        current_y = positions[current_idx, 1]
        current_heading = headings[current_idx]

        draw_robot(ax, current_x, current_y, current_heading, size=1.2,
                  color='#2E86AB', alpha=0.9)

        # Add distance traveled info
        distance_traveled = len(waypoints[:current_idx]) * 0.1  # 0.1m steps
        ax.text(0.02, 0.98, f'Distance: {distance_traveled:.1f}m',
               transform=ax.transAxes, fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        # Add heading info
        heading_deg = math.degrees(current_heading)
        ax.text(0.02, 0.85, f'Heading: {heading_deg:.1f}°',
               transform=ax.transAxes, fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

        # Add legend only to first subplot
        if idx == 0:
            ax.legend(loc='lower right', fontsize=9)

    # Adjust layout
    plt.tight_layout()

    # Save figure
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Navigation demo saved to: {output_path}")
    plt.close()


if __name__ == '__main__':
    # Save to docs/images directory
    output_file = Path(__file__).parent.parent.parent / 'docs' / 'images' / 'navigation_demo.png'
    output_file.parent.mkdir(parents=True, exist_ok=True)

    create_navigation_demo(str(output_file))
    print("Navigation demo completed!")
