"""
Real-time fleet dashboard for multi-AMR factory visualization.

Visualizes robot positions, tasks, stations, traffic, battery levels, and production metrics
using matplotlib with animation support. Professional portfolio-quality visualization.

Author: Muskaan Maheshwari
"""

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle, FancyArrowPatch, Polygon, Wedge
import matplotlib.lines as mlines
import numpy as np
from typing import Dict, List, Tuple, Optional, Any


class FleetDashboard:
    """
    Real-time fleet dashboard with factory floor map, status panels, and production metrics.

    Layout:
    ┌──────────────────────────────┬──────────────────┐
    │    Factory Floor Map         │  Fleet Status     │
    │    (main visualization)      │  (text panel)     │
    │                              ├──────────────────┤
    │                              │  Battery Levels   │
    │                              │  (horizontal bars)│
    ├──────────────────────────────┼──────────────────┤
    │  Task Timeline / Throughput  │  Production Stats │
    │  (line chart)                │  (text panel)     │
    └──────────────────────────────┴──────────────────┘
    """

    # Station type colors
    STATION_COLORS = {
        'INCOMING_MATERIAL': '#1E90FF',    # dodgerblue
        'CELL_ASSEMBLY': '#228B22',        # forestgreen
        'MODULE_PACKING': '#FF8C00',       # darkorange
        'PACK_INTEGRATION': '#9370DB',     # mediumpurple
        'TESTING_QC': '#DC143C',           # crimson
        'SHIPPING': '#8B4513',             # saddlebrown
        'CHARGING_STATION': '#FFD700',     # gold
        'PARKING_BAY': '#808080',          # gray
        'LOADING_DOCK': '#90EE90',         # lightgreen
        'UNLOADING_DOCK': '#FFB6C6',       # lightcoral
    }

    # Robot state colors
    ROBOT_STATE_COLORS = {
        'navigating': '#00CC44',           # green
        'idle': '#4488FF',                 # blue
        'charging': '#FFDD00',             # yellow
        'e_stop': '#FF0000',               # red
        'docking': '#FF8833',              # orange
        'parked': '#888888',               # gray
        'waiting': '#00DDDD',              # cyan
    }

    # Battery level colors
    BATTERY_COLORS = {
        'high': '#00CC44',                 # green (>50%)
        'medium': '#FFDD00',               # yellow (20-50%)
        'low': '#FF0000',                  # red (<20%)
    }

    def __init__(self, factory_env, robots: List, traffic_manager=None):
        """
        Initialize the fleet dashboard.

        Args:
            factory_env: Factory environment with stations and bounds
            robots: List of robot objects with state and position
            traffic_manager: Optional traffic manager for intersection data
        """
        self.factory_env = factory_env
        self.robots = robots
        self.traffic_manager = traffic_manager

        # Extract factory bounds and dimensions
        self.factory_bounds = getattr(factory_env, 'bounds', [(0, 0), (100, 100)])
        self.stations = getattr(factory_env, 'stations', {})
        self.aisles = getattr(factory_env, 'aisles', [])
        self.intersections = getattr(factory_env, 'intersections', [])
        self.restricted_zones = getattr(factory_env, 'restricted_zones', [])
        self.walls = getattr(factory_env, 'walls', [])

        # Production tracking
        self.task_history = []  # [(timestamp, task_count), ...]
        self.completion_times = []
        self.deadlock_count = 0

        # Alert tracking
        self.recent_alerts = []  # List of recent alert messages
        self.max_alerts_display = 5

        # Create figure and layout
        self.fig = plt.figure(figsize=(18, 10), constrained_layout=False)
        self.fig.patch.set_facecolor('white')

        # Use GridSpec for layout control with more rows for alerts
        gs = gridspec.GridSpec(3, 2, figure=self.fig, hspace=0.35, wspace=0.3)

        # Main floor map (large, left side, spans 3 rows)
        self.ax_floor = self.fig.add_subplot(gs[:, 0])
        self.ax_floor.set_aspect('equal')
        self.ax_floor.set_title('Factory Floor Map', fontsize=14, fontweight='bold')

        # Fleet status (top right)
        self.ax_status = self.fig.add_subplot(gs[0, 1])
        self.ax_status.axis('off')
        self.ax_status.set_title('Fleet Status', fontsize=12, fontweight='bold')

        # Battery levels (middle right)
        self.ax_battery = self.fig.add_subplot(gs[1, 1])
        self.ax_battery.set_title('Battery Levels', fontsize=11, fontweight='bold')
        self.ax_battery.set_xlim(0, 100)
        self.ax_battery.set_ylim(0, len(robots) + 0.5)
        self.ax_battery.set_xlabel('Battery %', fontsize=9)
        self.ax_battery.invert_yaxis()
        self.ax_battery.grid(axis='x', alpha=0.3)

        # Alert status panel (bottom right)
        self.ax_alerts = self.fig.add_subplot(gs[2, 1])
        self.ax_alerts.axis('off')
        self.ax_alerts.set_title('Recent Alerts', fontsize=11, fontweight='bold')
        self.alerts_text = self.ax_alerts.text(0.05, 0.95, 'No alerts', transform=self.ax_alerts.transAxes,
                                               fontsize=9, verticalalignment='top', family='monospace',
                                               bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.5))

        # Production metrics (middle right, now sharing with battery)
        self.ax_metrics = None  # Will use alerts instead

        # Task timeline (bottom left)
        self.ax_timeline = self.fig.add_subplot(gs[2, 0])
        self.ax_timeline.set_title('Task Throughput', fontsize=11, fontweight='bold')
        self.ax_timeline.set_xlabel('Time (s)', fontsize=9)
        self.ax_timeline.set_ylabel('Tasks/min', fontsize=9)
        self.ax_timeline.grid(alpha=0.3)

        # Artist containers for animations
        self.robot_circles = []
        self.robot_arrows = []
        self.robot_labels = []
        self.robot_paths = []
        self.robot_turrets = []

        # Initialize robot artists
        for i, robot in enumerate(robots):
            circle = Circle((0, 0), 0.4, color='blue', ec='black', linewidth=1)
            self.ax_floor.add_patch(circle)
            self.robot_circles.append(circle)

            arrow = FancyArrowPatch((0, 0), (0.5, 0), arrowstyle='->',
                                   mutation_scale=15, color='black', linewidth=1.5)
            self.ax_floor.add_patch(arrow)
            self.robot_arrows.append(arrow)

            label, = self.ax_floor.plot([], [], 'k', fontsize=8, fontweight='bold')
            self.robot_labels.append(label)

            path_line, = self.ax_floor.plot([], [], 'k--', linewidth=0.5, alpha=0.5)
            self.robot_paths.append(path_line)

            turret_line, = self.ax_floor.plot([], [], 'r-', linewidth=1)
            self.robot_turrets.append(turret_line)

        # Draw static floor map
        self.draw_factory_floor(self.ax_floor)

        # Text objects for status panels
        self.status_text = self.ax_status.text(0.05, 0.95, '', transform=self.ax_status.transAxes,
                                               fontsize=10, verticalalignment='top', family='monospace',
                                               bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))

        # Timeline data
        self.timeline_times = []
        self.timeline_throughput = []
        self.timeline_line, = self.ax_timeline.plot([], [], 'b-', linewidth=2, marker='o', markersize=4)

        # Timestamp for tracking
        self.current_time = 0.0
        self.frame_count = 0

    def draw_factory_floor(self, ax):
        """
        Draw the factory floor layout with stations, aisles, intersections, walls, and restricted zones.

        Args:
            ax: matplotlib axis to draw on
        """
        # Set axis limits from factory bounds
        if self.factory_bounds:
            bounds_x = [b[0] for b in self.factory_bounds]
            bounds_y = [b[1] for b in self.factory_bounds]
            margin = 5
            ax.set_xlim(min(bounds_x) - margin, max(bounds_x) + margin)
            ax.set_ylim(min(bounds_y) - margin, max(bounds_y) + margin)
        else:
            ax.set_xlim(-5, 105)
            ax.set_ylim(-5, 105)

        ax.set_xlabel('X (m)', fontsize=10)
        ax.set_ylabel('Y (m)', fontsize=10)
        ax.grid(True, alpha=0.2, linestyle=':')

        # Draw aisles (light gray bands)
        for aisle in self.aisles:
            x, y, width, height = aisle
            rect = Rectangle((x - width/2, y - height/2), width, height,
                            facecolor='#F5F5F5', edgecolor='gray', linewidth=0.5, alpha=0.6)
            ax.add_patch(rect)

        # Draw restricted zones with red hatching
        for zone in self.restricted_zones:
            x, y, width, height = zone
            rect = Rectangle((x - width/2, y - height/2), width, height,
                            facecolor='#FFE0E0', edgecolor='red', linewidth=1.5,
                            hatch='///', alpha=0.7)
            ax.add_patch(rect)

        # Draw walls as thick black lines
        for wall in self.walls:
            x1, y1, x2, y2 = wall
            ax.plot([x1, x2], [y1, y2], 'k-', linewidth=3)

        # Draw intersections as yellow circles
        for intersection in self.intersections:
            if isinstance(intersection, (tuple, list)):
                x, y = intersection[:2]
                circle = Circle((x, y), 0.5, facecolor='yellow', edgecolor='orange',
                              linewidth=1, alpha=0.7, zorder=2)
            else:
                # If it's an object with x, y attributes
                x, y = intersection.x, intersection.y
                circle = Circle((x, y), 0.5, facecolor='yellow', edgecolor='orange',
                              linewidth=1, alpha=0.7, zorder=2)
            ax.add_patch(circle)

        # Draw stations
        for station_name, station_data in self.stations.items():
            if isinstance(station_data, dict):
                x, y = station_data.get('position', (0, 0))
                station_type = station_data.get('type', 'INCOMING_MATERIAL')
                width = station_data.get('width', 3)
                height = station_data.get('height', 3)
            else:
                # If it's an object with attributes
                x, y = station_data.position if hasattr(station_data, 'position') else (0, 0)
                station_type = station_data.type if hasattr(station_data, 'type') else 'INCOMING_MATERIAL'
                width = station_data.width if hasattr(station_data, 'width') else 3
                height = station_data.height if hasattr(station_data, 'height') else 3

            color = self.STATION_COLORS.get(station_type, '#CCCCCC')

            # Draw station rectangle
            rect = Rectangle((x - width/2, y - height/2), width, height,
                            facecolor=color, edgecolor='black', linewidth=1.5, alpha=0.8)
            ax.add_patch(rect)

            # Draw special symbols for certain stations
            if station_type == 'CHARGING_STATION':
                # Add lightning bolt
                ax.text(x, y, '⚡', fontsize=12, ha='center', va='center', fontweight='bold')
            elif station_type == 'PARKING_BAY':
                # Add P
                ax.text(x, y, 'P', fontsize=10, ha='center', va='center',
                       fontweight='bold', color='white')
            elif station_type == 'LOADING_DOCK':
                # Green triangle
                triangle = Polygon([[x-0.3, y-0.3], [x+0.3, y-0.3], [x, y+0.3]],
                                 facecolor='darkgreen', edgecolor='black', linewidth=0.5)
                ax.add_patch(triangle)
            elif station_type == 'UNLOADING_DOCK':
                # Red triangle
                triangle = Polygon([[x-0.3, y-0.3], [x+0.3, y-0.3], [x, y+0.3]],
                                 facecolor='darkred', edgecolor='black', linewidth=0.5)
                ax.add_patch(triangle)

            # Station label (small, rotated)
            ax.text(x, y - height/2 - 0.7, station_name, fontsize=7, ha='center',
                   rotation=0, style='italic', color='black', bbox=dict(
                       boxstyle='round,pad=0.3', facecolor='white', alpha=0.7, edgecolor='none'))

    def add_obstacle(self, position: Tuple[float, float], radius: float = 0.3) -> None:
        """
        Add a dynamic obstacle to the visualization.

        Args:
            position: Obstacle position as (x, y).
            radius: Obstacle radius in meters.
        """
        circle = Circle(position, radius, facecolor='#FF0000', edgecolor='darkred',
                       linewidth=1.5, alpha=0.7, zorder=3)
        self.ax_floor.add_patch(circle)

    def update_alerts(self, alerts: List[Dict]) -> None:
        """
        Update alert display with recent alerts.

        Args:
            alerts: List of alert dictionaries with 'robot_id', 'event_type', 'severity', 'message'.
        """
        self.recent_alerts = alerts[-self.max_alerts_display:]

        if not self.recent_alerts:
            alert_text = "No active alerts"
        else:
            alert_lines = ["Recent Alerts:"]
            for alert in self.recent_alerts:
                robot_id = alert.get('robot_id', 'unknown')
                event = alert.get('event_type', 'unknown')
                severity = alert.get('severity', 'low')
                alert_lines.append(f"[{severity.upper()}] {robot_id}: {event}")

            alert_text = "\n".join(alert_lines)

        self.alerts_text.set_text(alert_text)
        self.fig.canvas.draw_idle()

    def update_frame(self, frame_data: Dict):
        """
        Update all dashboard elements with current fleet state.

        Args:
            frame_data: Dictionary from FleetCoordinator.get_fleet_status() containing:
                - robot_states: List of robot state dicts
                - tasks_active: Count of active tasks
                - traffic_data: Traffic manager data
                - production: Production metrics
        """
        self.frame_count += 1
        self.current_time = frame_data.get('timestamp', self.current_time)

        # Update robot positions and states
        robot_states = frame_data.get('robot_states', [])
        for i, robot in enumerate(self.robots):
            if i < len(robot_states):
                state = robot_states[i]

                # Extract position and state
                if isinstance(state, dict):
                    pos = state.get('position', (0, 0))
                    robot_state = state.get('state', 'idle')
                    battery = state.get('battery', 100)
                    heading = state.get('heading', 0)
                    path = state.get('path', [])
                    turret_angle = state.get('turret_angle', 0)
                else:
                    # Object with attributes
                    pos = state.position if hasattr(state, 'position') else (0, 0)
                    robot_state = state.state if hasattr(state, 'state') else 'idle'
                    battery = state.battery if hasattr(state, 'battery') else 100
                    heading = state.heading if hasattr(state, 'heading') else 0
                    path = state.path if hasattr(state, 'path') else []
                    turret_angle = state.turret_angle if hasattr(state, 'turret_angle') else 0

                # Update robot circle (position and color by state)
                color = self.ROBOT_STATE_COLORS.get(robot_state, '#4488FF')
                self.robot_circles[i].center = pos
                self.robot_circles[i].set_facecolor(color)
                self.robot_circles[i].set_zorder(5)

                # Update heading arrow
                arrow_length = 0.8
                dx = arrow_length * np.cos(np.radians(heading))
                dy = arrow_length * np.sin(np.radians(heading))
                self.robot_arrows[i].set_positions(pos, (pos[0] + dx, pos[1] + dy))

                # Update robot ID label
                label_text = f'R{i}'
                self.robot_labels[i].set_text(label_text)
                self.robot_labels[i].set_position((pos[0] + 0.6, pos[1] + 0.6))

                # Update path visualization
                if path and len(path) > 0:
                    path_array = np.array(path)
                    self.robot_paths[i].set_data(path_array[:, 0], path_array[:, 1])
                else:
                    self.robot_paths[i].set_data([], [])

                # Update turret direction (small line on top)
                turret_length = 0.3
                turret_dx = turret_length * np.cos(np.radians(turret_angle))
                turret_dy = turret_length * np.sin(np.radians(turret_angle))
                self.robot_turrets[i].set_data([pos[0], pos[0] + turret_dx],
                                               [pos[1], pos[1] + turret_dy])

        # Update battery levels
        self._update_battery_bars(robot_states)

        # Update fleet status text
        self._update_status_text(frame_data)

        # Update production metrics
        self._update_production_stats(frame_data)

        # Update timeline
        self._update_timeline(frame_data)

    def _update_battery_bars(self, robot_states: List):
        """Update battery level horizontal bar chart."""
        self.ax_battery.clear()
        self.ax_battery.set_xlim(0, 100)
        self.ax_battery.set_ylim(0, len(self.robots) + 0.5)
        self.ax_battery.set_xlabel('Battery %', fontsize=9)
        self.ax_battery.invert_yaxis()
        self.ax_battery.grid(axis='x', alpha=0.3)
        self.ax_battery.set_title('Battery Levels', fontsize=11, fontweight='bold')

        for i, state in enumerate(robot_states[:len(self.robots)]):
            battery = state.get('battery', 100) if isinstance(state, dict) else \
                     (state.battery if hasattr(state, 'battery') else 100)

            # Determine color based on battery level
            if battery > 50:
                bar_color = self.BATTERY_COLORS['high']
            elif battery > 20:
                bar_color = self.BATTERY_COLORS['medium']
            else:
                bar_color = self.BATTERY_COLORS['low']

            # Draw bar
            self.ax_battery.barh(i, battery, color=bar_color, edgecolor='black', linewidth=0.5)

            # Add percentage text
            self.ax_battery.text(battery + 2, i, f'{battery:.0f}%', va='center', fontsize=8)

            # Add robot label
            self.ax_battery.set_yticks(range(len(self.robots)))
            self.ax_battery.set_yticklabels([f'R{j}' for j in range(len(self.robots))], fontsize=8)

    def _update_status_text(self, frame_data: Dict):
        """Update fleet status text panel."""
        robot_states = frame_data.get('robot_states', [])

        # Count robots by state
        state_counts = {}
        for state in robot_states:
            robot_state = state.get('state', 'idle') if isinstance(state, dict) else \
                         (state.state if hasattr(state, 'state') else 'idle')
            state_counts[robot_state] = state_counts.get(robot_state, 0) + 1

        # Build status text
        status_lines = [
            f"Total Robots: {len(self.robots)}",
            f"",
            "State Distribution:",
        ]

        for state, count in sorted(state_counts.items()):
            status_lines.append(f"  {state.upper()}: {count}")

        # Traffic data
        traffic_data = frame_data.get('traffic_data', {})
        status_lines.extend([
            f"",
            "Traffic Status:",
            f"  Intersections: {traffic_data.get('occupied', 0)}/{traffic_data.get('total', 0)}",
            f"  Deadlocks: {traffic_data.get('deadlocks', 0)}",
        ])

        active_tasks = frame_data.get('tasks_active', 0)
        status_lines.append(f"")
        status_lines.append(f"Active Tasks: {active_tasks}")

        self.status_text.set_text('\n'.join(status_lines))

    def _update_production_stats(self, frame_data: Dict):
        """Update production metrics text panel."""
        metrics = frame_data.get('production', {})

        completed = metrics.get('tasks_completed', 0)
        avg_time = metrics.get('avg_task_time', 0)
        utilization = metrics.get('fleet_utilization', 0)
        throughput = metrics.get('throughput_tasks_per_hour', 0)
        deadlocks_resolved = metrics.get('deadlocks_resolved', 0)

        metrics_lines = [
            f"Tasks Completed: {completed}",
            f"",
            f"Avg Task Time: {avg_time:.1f}s",
            f"Fleet Utilization: {utilization:.1f}%",
            f"Throughput: {throughput:.2f} tasks/hr",
            f"",
            f"Deadlocks Resolved: {deadlocks_resolved}",
        ]

        self.metrics_text.set_text('\n'.join(metrics_lines))

    def _update_timeline(self, frame_data: Dict):
        """Update task throughput timeline chart."""
        metrics = frame_data.get('production', {})
        throughput = metrics.get('throughput_tasks_per_hour', 0)

        self.timeline_times.append(self.current_time)
        self.timeline_throughput.append(throughput)

        # Keep last 50 data points for smooth visualization
        if len(self.timeline_times) > 50:
            self.timeline_times = self.timeline_times[-50:]
            self.timeline_throughput = self.timeline_throughput[-50:]

        self.timeline_line.set_data(self.timeline_times, self.timeline_throughput)

        if len(self.timeline_times) > 0:
            max_time = max(self.timeline_times)
            min_time = min(self.timeline_times)
            self.ax_timeline.set_xlim(min_time, max_time + 1)

            max_throughput = max(self.timeline_throughput) if self.timeline_throughput else 10
            self.ax_timeline.set_ylim(0, max_throughput * 1.2)

    def create_animation(self, coordinator, total_steps: int = 500,
                        dt: float = 0.1, interval: int = 50) -> animation.FuncAnimation:
        """
        Create an animated dashboard that updates with fleet state.

        Args:
            coordinator: FleetCoordinator instance
            total_steps: Number of simulation frames to run
            dt: Time step for each frame (seconds)
            interval: Delay between frames in milliseconds

        Returns:
            matplotlib.animation.FuncAnimation object
        """
        def animate_frame(frame_num):
            # Update coordinator
            coordinator.update(dt)

            # Get current fleet status
            frame_data = coordinator.get_fleet_status()

            # Update dashboard
            self.update_frame(frame_data)

            return []

        anim = animation.FuncAnimation(self.fig, animate_frame, frames=total_steps,
                                      interval=interval, blit=False, repeat=False)
        return anim

    def save_snapshot(self, filename: str):
        """
        Save current dashboard state as PNG image.

        Args:
            filename: Path to save the PNG file
        """
        self.fig.savefig(filename, dpi=150, bbox_inches='tight', facecolor='white')
        print(f"Dashboard snapshot saved to {filename}")

    def show(self):
        """Display the interactive dashboard."""
        plt.show()

    @staticmethod
    def quick_snapshot(factory_env, robots: List, paths: Optional[List] = None,
                      title: Optional[str] = None) -> plt.Figure:
        """
        Create a one-shot snapshot visualization without animation.
        Useful for debugging and documentation.

        Args:
            factory_env: Factory environment
            robots: List of robot objects
            paths: Optional list of paths to visualize
            title: Optional title for the figure

        Returns:
            matplotlib.figure.Figure object
        """
        fig, ax = plt.subplots(figsize=(12, 10), constrained_layout=True)
        fig.patch.set_facecolor('white')
        ax.set_aspect('equal')

        if title:
            ax.set_title(title, fontsize=14, fontweight='bold')
        else:
            ax.set_title('Factory Floor Layout', fontsize=14, fontweight='bold')

        # Create temporary dashboard to use draw_factory_floor
        dashboard = FleetDashboard(factory_env, robots)
        dashboard.draw_factory_floor(ax)

        # Draw robots
        for i, robot in enumerate(robots):
            pos = robot.position if hasattr(robot, 'position') else (0, 0)
            state = robot.state if hasattr(robot, 'state') else 'idle'
            heading = robot.heading if hasattr(robot, 'heading') else 0

            color = FleetDashboard.ROBOT_STATE_COLORS.get(state, '#4488FF')
            circle = Circle(pos, 0.4, facecolor=color, edgecolor='black', linewidth=1.5)
            ax.add_patch(circle)

            # Heading arrow
            arrow_length = 0.8
            dx = arrow_length * np.cos(np.radians(heading))
            dy = arrow_length * np.sin(np.radians(heading))
            arrow = FancyArrowPatch(pos, (pos[0] + dx, pos[1] + dy),
                                  arrowstyle='->', mutation_scale=15,
                                  color='black', linewidth=1.5)
            ax.add_patch(arrow)

            # Robot label
            ax.text(pos[0] + 0.6, pos[1] + 0.6, f'R{i}', fontsize=9, fontweight='bold')

            # Draw paths if provided
            if paths and i < len(paths):
                path = paths[i]
                if path and len(path) > 0:
                    path_array = np.array(path)
                    ax.plot(path_array[:, 0], path_array[:, 1], 'k--', linewidth=0.8, alpha=0.6)

        # Create legend
        legend_elements = [
            mlines.Line2D([0], [0], marker='o', color='w', markerfacecolor='#00CC44',
                         markersize=8, label='Navigating', markeredgecolor='black'),
            mlines.Line2D([0], [0], marker='o', color='w', markerfacecolor='#4488FF',
                         markersize=8, label='Idle', markeredgecolor='black'),
            mlines.Line2D([0], [0], marker='o', color='w', markerfacecolor='#FFDD00',
                         markersize=8, label='Charging', markeredgecolor='black'),
            mlines.Line2D([0], [0], marker='o', color='w', markerfacecolor='#FF0000',
                         markersize=8, label='E-Stop', markeredgecolor='black'),
        ]
        ax.legend(handles=legend_elements, loc='upper right', fontsize=10)

        return fig
