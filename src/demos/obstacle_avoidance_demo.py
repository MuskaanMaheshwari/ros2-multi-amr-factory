"""
Obstacle Avoidance Visualization Demo

Demonstrates Dynamic Window Approach (DWA) obstacle avoidance in action.
Shows 4 frames depicting the complete avoidance sequence:
1. Robot approaching obstacle (DWA kicks in)
2. Robot emergency stop (obstacle too close)
3. Obstacle clear (monitoring)
4. Robot resume (new path calculated)

The DWA algorithm evaluates safe velocity commands by simulating forward motion
and checking collisions, selecting the best safe trajectory.

Author: Muskaan Maheshwari
"""

import sys
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Headless rendering
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Circle, Rectangle, Wedge
from matplotlib.transforms import Affine2D

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from factory.environment import FactoryEnvironment


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


def draw_obstacle(ax, x, y, radius=1.5, color='red', label=None):
    """
    Draw an obstacle as a circle.

    Args:
        ax: Matplotlib axes
        x, y: Obstacle position
        radius: Obstacle radius
        color: Obstacle color
        label: Text label
    """
    circle = Circle((x, y), radius, color=color, alpha=0.6, edgecolor='darkred',
                   linewidth=2, zorder=5)
    ax.add_patch(circle)

    if label:
        ax.text(x, y - radius - 1, label, ha='center', fontsize=10,
               fontweight='bold', color='darkred')


def draw_safety_zone(ax, x, y, radius=2.0):
    """
    Draw safety buffer zone around obstacle.

    Args:
        ax: Matplotlib axes
        x, y: Zone center
        radius: Safety radius
    """
    circle = Circle((x, y), radius, color='orange', alpha=0.2,
                   edgecolor='orange', linewidth=2, linestyle='--', zorder=3)
    ax.add_patch(circle)


def create_obstacle_avoidance_demo(output_path):
    """
    Create obstacle avoidance visualization showing DWA in action.

    Generates a 2x2 grid showing:
    Frame 1: Robot approaching obstacle (DWA kicks in)
    Frame 2: Robot emergency stop (obstacle too close)
    Frame 3: Obstacle clear (monitoring)
    Frame 4: Robot resume (new path calculated)

    Args:
        output_path: Path to save the PNG image
    """
    print("Creating obstacle avoidance demo...")

    # Create factory environment
    factory = FactoryEnvironment(width=100.0, height=80.0, resolution=0.5)

    # Define start and goal
    start_pos = (10.0, 40.0)
    goal_pos = (80.0, 40.0)
    obstacle_pos = (45.0, 40.0)  # Obstacle on direct path
    safety_radius = 1.5
    avoidance_radius = 2.0  # 1.5m safety buffer

    # Create 2x2 grid of frames
    fig, axes = plt.subplots(2, 2, figsize=(13, 11))
    fig.suptitle(
        'Dynamic Window Approach (DWA) - Obstacle Avoidance\n'
        'Real-time collision avoidance with emergency response',
        fontsize=14, fontweight='bold', y=0.98
    )

    # Frame data
    frames = [
        {
            'title': 'Frame 1: Approaching Obstacle\n(DWA evaluates safe paths)',
            'robot_x': 28.0,
            'robot_y': 40.0,
            'robot_heading': 0.0,
            'robot_color': '#2E86AB',
            'status': 'DWA ACTIVE',
            'status_color': 'orange',
            'path_x': [10, 28, 45],
            'path_y': [40, 40, 40],
            'obstacle_visible': True,
            'alert_msg': 'WARNING: Obstacle detected at 17m',
            'distance_to_obstacle': 17.0
        },
        {
            'title': 'Frame 2: Emergency Stop\n(obstacle too close)',
            'robot_x': 36.0,
            'robot_y': 40.0,
            'robot_heading': 0.0,
            'robot_color': '#FF6B6B',
            'status': 'EMERGENCY STOP',
            'status_color': 'red',
            'path_x': [10, 28, 36, 45],
            'path_y': [40, 40, 40, 40],
            'obstacle_visible': True,
            'alert_msg': 'CRITICAL: Emergency stop activated!',
            'distance_to_obstacle': 9.0
        },
        {
            'title': 'Frame 3: Obstacle Clear\n(new path calculated)',
            'robot_x': 36.0,
            'robot_y': 45.0,
            'robot_heading': math.pi / 2,
            'robot_color': '#FFD700',
            'status': 'REPLANNING',
            'status_color': 'blue',
            'path_x': [36, 36, 55],
            'path_y': [40, 45, 45],
            'obstacle_visible': True,
            'alert_msg': 'INFO: New path calculated',
            'distance_to_obstacle': None
        },
        {
            'title': 'Frame 4: Resume Navigation\n(obstacle cleared)',
            'robot_x': 70.0,
            'robot_y': 45.0,
            'robot_heading': 0.0,
            'robot_color': '#2E86AB',
            'status': 'NAVIGATING',
            'status_color': 'green',
            'path_x': [36, 55, 70, 80],
            'path_y': [45, 45, 45, 40],
            'obstacle_visible': False,
            'alert_msg': 'OK: Path clear, resuming',
            'distance_to_obstacle': None
        }
    ]

    for ax, frame_data in zip(axes.flat, frames):
        # Set up axes
        ax.set_xlim(5, 85)
        ax.set_ylim(30, 55)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_xlabel('X (m)', fontsize=10)
        ax.set_ylabel('Y (m)', fontsize=10)
        ax.set_title(frame_data['title'], fontsize=11, fontweight='bold')

        # Draw factory grid
        for i in range(10, 85, 10):
            ax.axvline(i, color='lightgray', linewidth=0.5, alpha=0.5)

        # Draw start and goal
        ax.plot(start_pos[0], start_pos[1], 'go', markersize=10,
               label='Start', zorder=4, markeredgecolor='black', markeredgewidth=1.5)
        ax.plot(goal_pos[0], goal_pos[1], 'r^', markersize=10,
               label='Goal', zorder=4, markeredgecolor='black', markeredgewidth=1.5)

        # Draw obstacle if visible
        if frame_data['obstacle_visible']:
            draw_obstacle(ax, obstacle_pos[0], obstacle_pos[1],
                         radius=safety_radius, color='red', label='Obstacle')
            draw_safety_zone(ax, obstacle_pos[0], obstacle_pos[1],
                           radius=avoidance_radius)

        # Draw path trajectory
        path_x = frame_data['path_x']
        path_y = frame_data['path_y']
        ax.plot(path_x, path_y, 'b--', linewidth=1.5, alpha=0.6, label='Path')

        # Draw robot
        draw_robot(ax, frame_data['robot_x'], frame_data['robot_y'],
                  frame_data['robot_heading'], size=1.2,
                  color=frame_data['robot_color'], alpha=0.9)

        # Add status box
        status_text = frame_data['status']
        ax.text(0.5, 0.95, status_text,
               transform=ax.transAxes, fontsize=11, fontweight='bold',
               ha='center', verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor=frame_data['status_color'],
                        alpha=0.7, edgecolor='black', linewidth=2))

        # Add distance info if available
        if frame_data['distance_to_obstacle'] is not None:
            distance_text = f"Distance to obstacle: {frame_data['distance_to_obstacle']:.1f}m"
            ax.text(0.02, 0.88, distance_text,
                   transform=ax.transAxes, fontsize=10, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

        # Add alert message box
        alert_msg = frame_data['alert_msg']
        msg_color = frame_data['status_color']
        ax.text(0.02, 0.05, f'ALERT: {alert_msg}',
               transform=ax.transAxes, fontsize=10, verticalalignment='bottom',
               bbox=dict(boxstyle='round', facecolor='lightyellow',
                        edgecolor=msg_color, linewidth=2, alpha=0.9),
               family='monospace', fontweight='bold')

        # Legend
        ax.legend(loc='upper right', fontsize=9)

    # Add a global info box at the bottom
    fig.text(0.5, 0.02,
            'Safety Distance: 1.5m buffer around obstacles | '
            'DWA evaluates safe velocities | '
            'Emergency stop < 1m approach distance',
            ha='center', fontsize=10, style='italic',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))

    # Adjust layout
    plt.tight_layout(rect=[0, 0.04, 1, 0.96])

    # Save figure
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Obstacle avoidance demo saved to: {output_path}")
    plt.close()


if __name__ == '__main__':
    # Save to docs/images directory
    output_file = Path(__file__).parent.parent.parent / 'docs' / 'images' / 'obstacle_avoidance_demo.png'
    output_file.parent.mkdir(parents=True, exist_ok=True)

    create_obstacle_avoidance_demo(str(output_file))
    print("Obstacle avoidance demo completed!")
