"""
Multi-frame navigation visualization for portfolio documentation.

Creates grid-based visualizations showing robot navigation progress at key
timepoints (0%, 25%, 50%, 75%, 100% of path). Includes progress indicators,
path visualization, and obstacle/station overlays.

Author: Muskaan Maheshwari
"""

import math
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle, Arrow, FancyArrowPatch, Polygon, Rectangle
import numpy as np


class NavigationAnimator:
    """
    Creates multi-frame visualization of navigation progress.

    Samples trajectory at progress milestones (0%, 25%, 50%, 75%, 100%) and
    creates subplot grid showing robot advancing along path. Includes:
      - Factory floor layout (walls, stations)
      - Planned path
      - Robot position and heading
      - Progress percentage overlay
      - Legend outside plot area

    Attributes:
        factory_env: Factory environment with stations, walls, bounds.
        robot_path: List of (x, y) waypoints along robot's path.
        robot_headings: List of heading angles (radians) corresponding to path points.
    """

    # Colors and styles
    PATH_COLOR = '#0066CC'
    ROBOT_COLOR = '#FF4400'
    STATION_COLOR = '#228B22'
    WALL_COLOR = '#333333'
    OBSTACLE_COLOR = '#FF0000'
    PROGRESS_COLOR = '#00CC44'

    def __init__(
        self,
        factory_env,
        robot_path: List[Tuple[float, float]],
        robot_headings: Optional[List[float]] = None,
    ) -> None:
        """
        Initialize navigation animator.

        Args:
            factory_env: Factory environment object with bounds, stations, walls.
            robot_path: List of (x, y) waypoints along navigation path.
            robot_headings: List of heading angles (radians) for each waypoint.
                           If None, computed from path direction.

        Example:
            animator = NavigationAnimator(factory_env, path, headings)
            fig = animator.create_grid_visualization(n_cols=2)
            plt.show()
        """
        self.factory_env = factory_env
        self.robot_path = robot_path
        self.robot_headings = robot_headings

        # Compute headings from path if not provided
        if self.robot_headings is None:
            self.robot_headings = self._compute_headings_from_path()

        # Extract factory bounds
        self.bounds = getattr(factory_env, 'bounds', [(0, 0), (100, 100)])
        self.min_x, self.min_y = self.bounds[0]
        self.max_x, self.max_y = self.bounds[1]

        # Extract features
        self.stations = getattr(factory_env, 'stations', {})
        self.walls = getattr(factory_env, 'walls', [])
        self.obstacles = getattr(factory_env, 'obstacles', [])

    def create_grid_visualization(
        self,
        n_cols: int = 2,
        robot_radius: float = 0.4,
        figsize: Optional[Tuple[int, int]] = None,
    ) -> plt.Figure:
        """
        Create multi-frame subplot grid showing navigation progress.

        Samples path at milestones: [0%, 25%, 50%, 75%, 100%].
        Arranges in grid with specified columns.

        Args:
            n_cols: Number of columns in subplot grid (default 2).
            robot_radius: Radius of robot circle visualization in meters.
            figsize: Figure size as (width, height) in inches (default auto).

        Returns:
            Matplotlib figure object with subplots.

        Example:
            fig = animator.create_grid_visualization(n_cols=2)
            fig.savefig('navigation_progress.png', dpi=150, bbox_inches='tight')
            plt.show()
        """
        # Determine frame points at progress milestones
        progress_points = [0.0, 0.25, 0.5, 0.75, 1.0]
        frame_indices = [
            int(p * (len(self.robot_path) - 1))
            for p in progress_points
        ]

        # Create subplot grid
        n_frames = len(frame_indices)
        n_rows = (n_frames + n_cols - 1) // n_cols  # Ceiling division

        if figsize is None:
            figsize = (5 * n_cols, 5 * n_rows)

        fig, axes = plt.subplots(
            n_rows, n_cols,
            figsize=figsize,
            constrained_layout=False
        )

        # Ensure axes is always 2D
        if n_frames == 1:
            axes = [[axes]]
        elif n_rows == 1 or n_cols == 1:
            axes = [axes] if n_rows == 1 else [[ax] for ax in axes]
        else:
            axes = axes

        # Draw each frame
        for frame_num, idx in enumerate(frame_indices):
            row = frame_num // n_cols
            col = frame_num % n_cols
            ax = axes[row][col]

            progress_pct = progress_points[frame_num] * 100
            self._draw_frame(
                ax, idx, robot_radius, progress_pct
            )

        # Hide unused subplots
        for frame_num in range(n_frames, n_rows * n_cols):
            row = frame_num // n_cols
            col = frame_num % n_cols
            axes[row][col].set_visible(False)

        # Add legend outside all subplots
        self._add_legend(fig, axes, n_rows, n_cols)

        fig.patch.set_facecolor('white')

        return fig

    def _draw_frame(
        self,
        ax: plt.Axes,
        waypoint_idx: int,
        robot_radius: float,
        progress_pct: float,
    ) -> None:
        """
        Draw a single frame of navigation progress.

        Args:
            ax: Matplotlib axes to draw on.
            waypoint_idx: Index of waypoint to show robot at.
            robot_radius: Radius of robot visualization.
            progress_pct: Progress percentage for title.
        """
        # Set axis properties
        ax.set_xlim(self.min_x - 2, self.max_x + 2)
        ax.set_ylim(self.min_y - 2, self.max_y + 2)
        ax.set_aspect('equal')
        ax.set_facecolor('#F0F0F0')

        # Draw factory floor
        self._draw_walls(ax)
        self._draw_stations(ax)
        self._draw_obstacles(ax)

        # Draw path
        self._draw_path(ax, waypoint_idx)

        # Draw robot at this waypoint
        if 0 <= waypoint_idx < len(self.robot_path):
            x, y = self.robot_path[waypoint_idx]
            heading = self.robot_headings[waypoint_idx] if waypoint_idx < len(self.robot_headings) else 0.0
            self._draw_robot(ax, x, y, heading, robot_radius)

        # Add progress indicator
        ax.text(
            0.5, 0.02,
            f"Progress: {progress_pct:.0f}%",
            transform=ax.transAxes,
            ha='center', va='bottom',
            fontsize=12, weight='bold',
            bbox=dict(boxstyle='round', facecolor=self.PROGRESS_COLOR, alpha=0.7)
        )

        # Title
        ax.set_title(f"Navigation Progress {progress_pct:.0f}%", fontsize=12, weight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')

    def _draw_walls(self, ax: plt.Axes) -> None:
        """Draw wall obstacles."""
        for wall in self.walls:
            if len(wall) == 4:  # Assume [x1, y1, x2, y2]
                x1, y1, x2, y2 = wall
                ax.plot([x1, x2], [y1, y2], color=self.WALL_COLOR, linewidth=3, zorder=1)
            elif len(wall) >= 2:  # Polygon
                wall_array = np.array(wall)
                poly = Polygon(wall_array, closed=True, facecolor='gray', alpha=0.3, edgecolor=self.WALL_COLOR)
                ax.add_patch(poly)

    def _draw_stations(self, ax: plt.Axes) -> None:
        """Draw station locations."""
        for station_id, station_info in self.stations.items():
            if 'position' in station_info:
                x, y = station_info['position']
                station_type = station_info.get('type', 'unknown')

                # Draw station circle
                circle = Circle((x, y), 0.5, color=self.STATION_COLOR, alpha=0.5, zorder=2)
                ax.add_patch(circle)

                # Label
                ax.text(x, y, station_id[:3], ha='center', va='center', fontsize=8, weight='bold')

    def _draw_obstacles(self, ax: plt.Axes) -> None:
        """Draw detected obstacles."""
        for obstacle in self.obstacles:
            if isinstance(obstacle, tuple) and len(obstacle) >= 2:
                x, y = obstacle[:2]
                radius = obstacle[2] if len(obstacle) > 2 else 0.3

                circle = Circle((x, y), radius, color=self.OBSTACLE_COLOR, alpha=0.6, zorder=2)
                ax.add_patch(circle)

    def _draw_path(self, ax: plt.Axes, current_idx: int) -> None:
        """
        Draw navigation path, highlighting completed portion.

        Args:
            ax: Matplotlib axes.
            current_idx: Current waypoint index.
        """
        if len(self.robot_path) < 2:
            return

        # Draw full path (light)
        path_array = np.array(self.robot_path)
        ax.plot(path_array[:, 0], path_array[:, 1], color=self.PATH_COLOR, linewidth=1.5, alpha=0.3, zorder=3, label='Full path')

        # Draw completed path (dark)
        if current_idx > 0:
            completed_array = np.array(self.robot_path[:current_idx + 1])
            ax.plot(completed_array[:, 0], completed_array[:, 1], color=self.PATH_COLOR, linewidth=2.5, alpha=0.8, zorder=4, label='Completed path')

        # Mark start and goal
        start_x, start_y = self.robot_path[0]
        ax.plot(start_x, start_y, 'go', markersize=8, zorder=5, label='Start')

        if len(self.robot_path) > 1:
            goal_x, goal_y = self.robot_path[-1]
            ax.plot(goal_x, goal_y, 'r*', markersize=15, zorder=5, label='Goal')

    def _draw_robot(
        self,
        ax: plt.Axes,
        x: float,
        y: float,
        heading: float,
        radius: float,
    ) -> None:
        """
        Draw robot at position with heading indicator.

        Args:
            ax: Matplotlib axes.
            x: Robot x position.
            y: Robot y position.
            heading: Robot heading in radians.
            radius: Robot radius in meters.
        """
        # Robot body
        circle = Circle((x, y), radius, color=self.ROBOT_COLOR, alpha=0.8, zorder=6)
        ax.add_patch(circle)

        # Heading arrow
        arrow_length = radius * 1.5
        arrow_end_x = x + arrow_length * math.cos(heading)
        arrow_end_y = y + arrow_length * math.sin(heading)

        arrow = FancyArrowPatch(
            (x, y), (arrow_end_x, arrow_end_y),
            arrowstyle='->', mutation_scale=20, color='white',
            linewidth=2, zorder=7
        )
        ax.add_patch(arrow)

        # Label
        ax.text(x, y - radius - 0.5, 'AMR', ha='center', fontsize=9, weight='bold')

    def _add_legend(
        self,
        fig: plt.Figure,
        axes,
        n_rows: int,
        n_cols: int,
    ) -> None:
        """
        Add legend outside the subplot grid.

        Args:
            fig: Matplotlib figure.
            axes: Subplot axes array.
            n_rows: Number of subplot rows.
            n_cols: Number of subplot columns.
        """
        # Collect legend handles and labels from first subplot
        if n_rows > 0 and n_cols > 0:
            first_ax = axes[0][0]
            handles, labels = first_ax.get_legend_handles_labels()

            if handles:
                fig.legend(
                    handles, labels,
                    loc='lower center',
                    bbox_to_anchor=(0.5, -0.05),
                    ncol=4,
                    frameon=True,
                    fancybox=True,
                    shadow=True,
                )

    def _compute_headings_from_path(self) -> List[float]:
        """
        Compute heading angles from path waypoints.

        Uses direction between consecutive waypoints.

        Returns:
            List of heading angles in radians.
        """
        if len(self.robot_path) < 2:
            return [0.0] * len(self.robot_path)

        headings = []
        for i in range(len(self.robot_path) - 1):
            x1, y1 = self.robot_path[i]
            x2, y2 = self.robot_path[i + 1]

            heading = math.atan2(y2 - y1, x2 - x1)
            headings.append(heading)

        # Use last heading for final point
        headings.append(headings[-1])

        return headings
