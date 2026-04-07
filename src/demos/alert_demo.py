"""
Multi-Robot Alert System Visualization Demo

Demonstrates the fleet coordination and alert broadcasting system.
Shows 3 robots navigating with real-time alert handling:
- Robot 1: Normal operation
- Robot 2: Encounters obstacle, generates alert, stops
- Robot 3: Normal operation, avoids Robot 2's area due to alert broadcast

Timeline of events:
- t=0s: All robots start
- t=3s: Robot 2 detects obstacle
- t=3.1s: Alert sent to hub and broadcast to fleet
- t=3.5s: Robot 3 receives alert, reroutes
- t=5s: Obstacle cleared, Robot 2 resumes

The alert system enables dynamic fleet coordination without direct robot-to-robot
communication - all coordination flows through a central hub.

Author: Muskaan Maheshwari
"""

import sys
import math
from pathlib import Path
from typing import List, Tuple
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Headless rendering
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Circle, Rectangle, FancyBboxPatch
from matplotlib.transforms import Affine2D
from matplotlib.collections import PatchCollection

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from factory.environment import FactoryEnvironment


def draw_robot(ax, x, y, heading, robot_id, color, size=1.2, alpha=0.9):
    """
    Draw a robot with ID label.

    Args:
        ax: Matplotlib axes
        x, y: Robot position
        heading: Robot heading in radians
        robot_id: Robot identifier
        color: Robot color
        size: Robot size
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
    arrow_length = size * 1.3
    dx = arrow_length * math.cos(heading)
    dy = arrow_length * math.sin(heading)

    arrow = FancyArrowPatch(
        (x, y), (x + dx, y + dy),
        arrowstyle='->', mutation_scale=18,
        color=color, linewidth=2, alpha=alpha
    )
    ax.add_patch(arrow)

    # Robot ID label
    ax.text(x, y - size/2 - 2, f'R{robot_id}', ha='center', fontsize=9,
           fontweight='bold', color=color,
           bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))


def draw_obstacle(ax, x, y, radius=1.5):
    """Draw an obstacle circle."""
    circle = Circle((x, y), radius, color='red', alpha=0.6,
                   edgecolor='darkred', linewidth=2, zorder=5)
    ax.add_patch(circle)


def draw_exclusion_zone(ax, x, y, radius=3.0):
    """Draw an alert exclusion zone around danger area."""
    circle = Circle((x, y), radius, color='orange', alpha=0.15,
                   edgecolor='orange', linewidth=2, linestyle='--', zorder=2)
    ax.add_patch(circle)


def draw_alert_box(ax, messages: List[str], y_pos=0.95):
    """
    Draw alert messages box.

    Args:
        ax: Matplotlib axes
        messages: List of alert messages
        y_pos: Y position in axes coordinates
    """
    if not messages:
        return

    msg_text = '\n'.join(messages)
    ax.text(0.98, y_pos, msg_text,
           transform=ax.transAxes, fontsize=9, verticalalignment='top',
           ha='right', family='monospace',
           bbox=dict(boxstyle='round', facecolor='#FFE6E6',
                    edgecolor='red', linewidth=2, alpha=0.95))


def create_timeline_event(event_time, robot_id, event_type, message):
    """Create a timeline event dictionary."""
    return {
        'time': event_time,
        'robot_id': robot_id,
        'type': event_type,  # 'start', 'detect', 'alert_send', 'alert_receive', 'resume', 'clear'
        'message': message
    }


def create_alert_demo(output_path):
    """
    Create multi-robot alert system visualization.

    Shows how alerts propagate through the fleet and how other robots
    adjust their paths based on broadcast alerts.

    Args:
        output_path: Path to save the PNG image
    """
    print("Creating alert system demo...")

    # Create factory environment
    factory = FactoryEnvironment(width=100.0, height=80.0, resolution=0.5)

    # Define robot initial positions and goals
    robots_config = {
        1: {'start': (15, 20), 'goal': (75, 25), 'color': '#2E86AB'},
        2: {'start': (15, 40), 'goal': (75, 40), 'color': '#FF6B6B'},
        3: {'start': (15, 60), 'goal': (75, 55), 'color': '#51CF66'}
    }

    obstacle_pos = (45, 40)  # Obstacle on Robot 2's path

    # Create timeline of events
    events = [
        create_timeline_event(0.0, 1, 'start', 'Robot 1 navigating'),
        create_timeline_event(0.0, 2, 'start', 'Robot 2 navigating'),
        create_timeline_event(0.0, 3, 'start', 'Robot 3 navigating'),
        create_timeline_event(3.0, 2, 'detect', 'Obstacle detected!'),
        create_timeline_event(3.1, 2, 'alert_send', 'Alert sent to hub'),
        create_timeline_event(3.2, 2, 'alert_send', 'Broadcast to fleet'),
        create_timeline_event(3.5, 3, 'alert_receive', 'Alert received, rerouting'),
        create_timeline_event(5.0, 2, 'resume', 'Obstacle cleared, resuming'),
    ]

    # Create visualization panels
    fig = plt.figure(figsize=(15, 10))
    gs = fig.add_gridspec(3, 2, hspace=0.35, wspace=0.3)

    # Main timeline visualization spanning both columns
    ax_main = fig.add_subplot(gs[0:2, :])

    # Timeline event log (bottom)
    ax_timeline = fig.add_subplot(gs[2, :])

    fig.suptitle(
        'Multi-Robot Alert System - Fleet Coordination\n'
        'Real-time alert propagation and dynamic path adjustment',
        fontsize=14, fontweight='bold', y=0.98
    )

    # ===== Main Fleet Visualization =====
    ax_main.set_xlim(5, 80)
    ax_main.set_ylim(10, 70)
    ax_main.set_aspect('equal')
    ax_main.grid(True, alpha=0.3, linestyle='--')
    ax_main.set_xlabel('X (m)', fontsize=11)
    ax_main.set_ylabel('Y (m)', fontsize=11)
    ax_main.set_title('Fleet State at t=3.5s (after Robot 3 reroutes)', fontsize=12, fontweight='bold')

    # Draw factory grid
    for i in range(10, 80, 10):
        ax_main.axvline(i, color='lightgray', linewidth=0.5, alpha=0.5)
        ax_main.axhline(i, color='lightgray', linewidth=0.5, alpha=0.5)

    # Robot 1: Normal operation, continuous progress
    robot1_pos = (35, 20)
    draw_robot(ax_main, robot1_pos[0], robot1_pos[1], 0.0, 1,
              robots_config[1]['color'], size=1.2)
    ax_main.plot([robots_config[1]['start'][0], robot1_pos[0]],
                 [robots_config[1]['start'][1], robot1_pos[1]],
                 color=robots_config[1]['color'], linewidth=2, alpha=0.6,
                 label='Robot 1 (normal)')
    ax_main.text(robot1_pos[0], robot1_pos[1] - 4, 'NAVIGATING',
                fontsize=9, ha='center', color=robots_config[1]['color'],
                fontweight='bold')

    # Robot 2: Stopped due to obstacle
    robot2_pos = (38, 40)
    draw_robot(ax_main, robot2_pos[0], robot2_pos[1], 0.0, 2,
              robots_config[2]['color'], size=1.2)
    ax_main.plot([robots_config[2]['start'][0], robot2_pos[0]],
                 [robots_config[2]['start'][1], robot2_pos[1]],
                 color=robots_config[2]['color'], linewidth=2, alpha=0.6,
                 label='Robot 2 (obstacle)')
    ax_main.text(robot2_pos[0], robot2_pos[1] - 4, 'E-STOP',
                fontsize=9, ha='center', color=robots_config[2]['color'],
                fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    # Robot 3: Diverted due to alert
    robot3_pos = (40, 62)
    robot3_heading = math.pi / 2  # Heading up
    draw_robot(ax_main, robot3_pos[0], robot3_pos[1], robot3_heading, 3,
              robots_config[3]['color'], size=1.2)
    # Original path
    ax_main.plot([robots_config[3]['start'][0], obstacle_pos[0]],
                 [robots_config[3]['start'][1], obstacle_pos[1]],
                 color=robots_config[3]['color'], linewidth=1.5,
                 linestyle='--', alpha=0.4, label='Original path (avoided)')
    # Rerouted path
    ax_main.plot([robots_config[3]['start'][0], 40, robot3_pos[0]],
                 [robots_config[3]['start'][1], 62, robot3_pos[1]],
                 color=robots_config[3]['color'], linewidth=2, alpha=0.7,
                 label='Robot 3 (rerouted)')
    ax_main.text(robot3_pos[0], robot3_pos[1] + 3, 'REROUTED',
                fontsize=9, ha='center', color=robots_config[3]['color'],
                fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))

    # Obstacle
    draw_obstacle(ax_main, obstacle_pos[0], obstacle_pos[1], radius=1.5)
    draw_exclusion_zone(ax_main, obstacle_pos[0], obstacle_pos[1], radius=3.0)
    ax_main.text(obstacle_pos[0], obstacle_pos[1] - 4, 'OBSTACLE',
                fontsize=9, ha='center', color='red', fontweight='bold')

    # Draw goal positions
    for rid, config in robots_config.items():
        ax_main.plot(config['goal'][0], config['goal'][1], '^', color=config['color'],
                    markersize=10, markeredgecolor='black', markeredgewidth=1.5)

    # Alert broadcast visualization
    ax_main.text(obstacle_pos[0], obstacle_pos[1] + 5, 'ALERT\nBROADCAST',
                fontsize=10, ha='center', fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='#FFE6E6',
                         edgecolor='red', linewidth=2))

    # Arrows showing alert propagation
    for robot_id in [1, 3]:
        start_pos = (obstacle_pos[0], obstacle_pos[1] + 5)
        if robot_id == 1:
            end_pos = (robot1_pos[0] - 2, robot1_pos[1] + 2)
        else:
            end_pos = (robot3_pos[0] + 2, robot3_pos[1] - 2)

        arrow = FancyArrowPatch(
            start_pos, end_pos,
            arrowstyle='->', mutation_scale=20, color='orange',
            linewidth=2, linestyle='--', alpha=0.6
        )
        ax_main.add_patch(arrow)

    # Legend
    ax_main.legend(loc='upper left', fontsize=10, framealpha=0.95)

    # ===== Timeline Events =====
    ax_timeline.axis('off')
    ax_timeline.set_xlim(0, 10)
    ax_timeline.set_ylim(0, 3)

    # Draw timeline
    timeline_y = 1.5
    ax_timeline.arrow(0.5, timeline_y, 9, 0, head_width=0.2, head_length=0.2,
                     fc='black', ec='black', linewidth=2)

    # Event markers and labels
    time_positions = {
        0.0: 0.5,
        3.0: 4.0,
        3.1: 4.2,
        3.5: 4.8,
        5.0: 6.5,
    }

    event_colors = {
        'start': 'green',
        'detect': 'orange',
        'alert_send': 'red',
        'alert_receive': 'blue',
        'resume': 'green',
        'clear': 'green'
    }

    for event in events:
        if event['time'] in time_positions:
            x_pos = time_positions[event['time']]
            color = event_colors.get(event['type'], 'gray')

            # Time label
            ax_timeline.text(x_pos, timeline_y - 0.4, f"t={event['time']}s",
                           ha='center', fontsize=9, fontweight='bold')

            # Event marker
            marker_size = 10 if event['type'] in ['detect', 'alert_send', 'alert_receive'] else 8
            ax_timeline.plot(x_pos, timeline_y, 'o', color=color, markersize=marker_size,
                           markeredgecolor='black', markeredgewidth=1.5, zorder=5)

            # Event description
            desc = f"R{event['robot_id']}: {event['message']}"
            ax_timeline.text(x_pos, timeline_y + 0.6, desc, ha='center', fontsize=8,
                           rotation=0, bbox=dict(boxstyle='round,pad=0.3',
                                               facecolor='lightyellow', alpha=0.8))

    ax_timeline.text(0.5, 2.5, 'EVENT TIMELINE', fontsize=11, fontweight='bold')

    # Add alert box with system status
    alert_msgs = [
        'ALERT SYSTEM STATUS:',
        't=3.0s: Robot 2 detects obstacle',
        't=3.1s: Alert broadcasted to hub',
        't=3.2s: Alert sent to all robots',
        't=3.5s: Robot 3 receives & reroutes',
        't=5.0s: Obstacle cleared',
        ''
        'Result: Zero collision, optimal avoidance'
    ]
    draw_alert_box(ax_main, alert_msgs, y_pos=0.98)

    # Add info at bottom
    fig.text(0.5, 0.01,
            'Alert System enables dynamic fleet coordination | '
            'Exclusion zones prevent robot-obstacle collisions | '
            'Rerouting algorithms minimize path delays',
            ha='center', fontsize=10, style='italic',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))

    # Adjust layout
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])

    # Save figure
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Alert demo saved to: {output_path}")
    plt.close()


if __name__ == '__main__':
    # Save to docs/images directory
    output_file = Path(__file__).parent.parent.parent / 'docs' / 'images' / 'alert_demo.png'
    output_file.parent.mkdir(parents=True, exist_ok=True)

    create_alert_demo(str(output_file))
    print("Alert demo completed!")
