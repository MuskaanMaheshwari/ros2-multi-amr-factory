#!/usr/bin/env python3
"""
Animated GIF generators for Multi-AMR Factory Navigation.

Creates two publication-quality animated GIFs:
  1. Fleet navigation — 3 AMRs following Dubins paths through factory
  2. Obstacle avoidance — AMR detects person, stops, alerts hub, resumes

These GIFs demonstrate the system for GitHub portfolio display.

Author: Muskaan Maheshwari
"""

import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from PIL import Image
import io

# Ensure src is importable
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from src.factory.environment import FactoryEnvironment, StationType
from src.amr.robot import AMRRobot, AMRState
from src.planning.dubins import DubinsPlanner


# ---------------------------------------------------------------------------
# Helper: render one frame of the factory with robots, paths, obstacles
# ---------------------------------------------------------------------------

def _render_frame(
    factory: FactoryEnvironment,
    robots: list,
    planned_paths: dict = None,
    trail_history: dict = None,
    obstacles: list = None,
    alert_text: str = None,
    status_text: str = None,
    title: str = "Multi-AMR Factory Navigation",
) -> Image.Image:
    """Render a single animation frame and return as PIL Image.

    Args:
        factory: The factory environment.
        robots: List of dicts with 'id', 'x', 'y', 'heading', 'color', 'state'.
        planned_paths: Dict mapping robot_id to full planned path [(x,y), ...].
        trail_history: Dict mapping robot_id to positions visited so far.
        obstacles: List of dicts with 'x', 'y', 'radius', 'label'.
        alert_text: Text to display in alert box (bottom-left).
        status_text: Text for top-right status box.
        title: Figure title.

    Returns:
        PIL Image of the rendered frame.
    """
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_xlim(0, factory.width)
    ax.set_ylim(0, factory.height)
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.set_xlabel("X (meters)", fontsize=9)
    ax.set_ylabel("Y (meters)", fontsize=9)
    ax.set_title(title, fontsize=13, fontweight="bold")

    # Station colors
    station_colors = {
        StationType.INCOMING_MATERIAL: "#FF6B6B",
        StationType.CELL_ASSEMBLY: "#4ECDC4",
        StationType.MODULE_PACKING: "#45B7D1",
        StationType.PACK_INTEGRATION: "#96CEB4",
        StationType.TESTING_QC: "#FFEAA7",
        StationType.SHIPPING: "#DDA15E",
        StationType.CHARGING_STATION: "#BC6C25",
        StationType.PARKING_BAY: "#8E7DBE",
    }

    # Draw aisles
    for y in [10.0, 20.0, 35.0, 50.0, 65.0, 75.0]:
        ax.add_patch(mpatches.Rectangle((0, y - 2), factory.width, 4, lw=0, fc="#E8E8E8", zorder=0))
    for x in [12.0, 37.0, 58.0, 77.0]:
        ax.add_patch(mpatches.Rectangle((x - 1.5, 0), 3, factory.height, lw=0, fc="#E8E8E8", zorder=0))

    # Draw restricted corners
    for cx, cy in [(0, 0), (96, 0), (0, 76), (96, 76)]:
        ax.add_patch(mpatches.Rectangle((cx, cy), 4, 4, lw=0.5, fc="red", alpha=0.25, hatch="///", zorder=1))

    # Draw intersections
    for ix, iy in factory.get_intersections():
        ax.plot(ix, iy, "D", ms=5, color="gold", mec="black", mew=0.5, zorder=2)

    # Draw stations (simplified for GIF — smaller labels)
    for st in factory.stations.values():
        x, y = st.position
        w, h = st.width, st.length
        c = station_colors.get(st.station_type, "#CCC")
        ax.add_patch(mpatches.FancyBboxPatch(
            (x - w/2, y - h/2), w, h,
            boxstyle="round,pad=0.05", lw=0.8, ec="black", fc=c, alpha=0.7, zorder=3,
        ))
        # Compact label
        short = st.station_id.split("_")[0][:4]
        ax.text(x, y, short, ha="center", va="center", fontsize=5, zorder=4)

    # Draw planned paths (dashed, faint)
    if planned_paths:
        for rid, ppath in planned_paths.items():
            if len(ppath) > 1:
                xs = [p[0] for p in ppath]
                ys = [p[1] for p in ppath]
                ax.plot(xs, ys, "--", color="#888", lw=1, alpha=0.4, zorder=5)

    # Draw trail history (solid, coloured)
    if trail_history:
        for rid, trail in trail_history.items():
            if len(trail) > 1:
                # Find matching robot color
                rc = "#2E86AB"
                for r in robots:
                    if r["id"] == rid:
                        rc = r.get("color", "#2E86AB")
                        break
                xs = [p[0] for p in trail]
                ys = [p[1] for p in trail]
                ax.plot(xs, ys, "-", color=rc, lw=2, alpha=0.7, zorder=6)

    # Draw obstacles
    if obstacles:
        for obs in obstacles:
            # Safety zone
            ax.add_patch(mpatches.Circle(
                (obs["x"], obs["y"]), obs.get("radius", 1.5),
                fc="red", alpha=0.15, ec="red", lw=1.5, ls="--", zorder=7,
            ))
            # Obstacle marker
            ax.plot(obs["x"], obs["y"], "X", ms=14, color="red", mec="darkred", mew=2, zorder=8)
            if obs.get("label"):
                ax.text(obs["x"], obs["y"] - 2.0, obs["label"],
                        ha="center", fontsize=8, color="red", fontweight="bold", zorder=9)

    # Draw robots
    for r in robots:
        rx, ry, rh = r["x"], r["y"], r["heading"]
        color = r.get("color", "#2E86AB")
        state = r.get("state", "navigating")

        # Outer circle with state-dependent edge
        ec = "red" if state == "emergency_stop" else "black"
        lw = 2.5 if state == "emergency_stop" else 1.5
        ax.add_patch(mpatches.Circle((rx, ry), 1.0, fc=color, ec=ec, lw=lw, zorder=10))

        # Heading arrow
        dx, dy = 1.2 * math.cos(rh), 1.2 * math.sin(rh)
        ax.annotate("", xy=(rx + dx, ry + dy), xytext=(rx, ry),
                     arrowprops=dict(arrowstyle="->", color="white", lw=2), zorder=11)

        # Label
        ax.text(rx, ry + 1.8, r["id"], ha="center", fontsize=7, fontweight="bold",
                color=color, zorder=12)

    # Alert text box (bottom-left)
    if alert_text:
        props = dict(boxstyle="round,pad=0.5", facecolor="#FFDDDD", edgecolor="red", alpha=0.9)
        ax.text(2, factory.height - 3, alert_text, fontsize=8, va="top",
                bbox=props, zorder=15, family="monospace")

    # Status text box (top-right, outside factory but inside figure)
    if status_text:
        props = dict(boxstyle="round,pad=0.4", facecolor="#DDFFDD", edgecolor="green", alpha=0.9)
        ax.text(factory.width - 2, 3, status_text, fontsize=8, va="top", ha="right",
                bbox=props, zorder=15, family="monospace")

    # Compact legend outside plot
    legend_elems = [
        mpatches.Patch(fc=station_colors[StationType.INCOMING_MATERIAL], label="Incoming"),
        mpatches.Patch(fc=station_colors[StationType.CELL_ASSEMBLY], label="Assembly"),
        mpatches.Patch(fc=station_colors[StationType.MODULE_PACKING], label="Packing"),
        mpatches.Patch(fc=station_colors[StationType.PACK_INTEGRATION], label="Integration"),
        mpatches.Patch(fc=station_colors[StationType.TESTING_QC], label="Testing/QC"),
        mpatches.Patch(fc=station_colors[StationType.SHIPPING], label="Shipping"),
        mpatches.Patch(fc=station_colors[StationType.CHARGING_STATION], label="Charging"),
        mpatches.Patch(fc=station_colors[StationType.PARKING_BAY], label="Parking"),
    ]
    ax.legend(handles=legend_elems, loc="upper left", bbox_to_anchor=(1.01, 1.0),
              fontsize=7, framealpha=0.9, borderaxespad=0)

    ax.grid(True, alpha=0.15, ls=":")
    fig.tight_layout(rect=[0, 0, 0.87, 1])

    # Convert to PIL Image
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=100, bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    return Image.open(buf).copy()


# ---------------------------------------------------------------------------
# GIF 1: Fleet Navigation — 3 AMRs following Dubins paths
# ---------------------------------------------------------------------------

def create_fleet_navigation_gif(output_path: str, n_frames: int = 60) -> str:
    """Create animated GIF of 3 AMRs navigating through the factory.

    Three robots start at parking bays and navigate to different production
    stations via Dubins curved paths. The animation shows:
      - Planned paths (dashed grey)
      - Real-time trail (colored solid line)
      - Robot position and heading at each frame
      - Station approach and docking

    Args:
        output_path: File path for the output GIF.
        n_frames: Number of animation frames.

    Returns:
        Path to the created GIF.
    """
    print("  Creating factory environment...")
    factory = FactoryEnvironment()
    dubins = DubinsPlanner(turning_radius=3.0)

    # Define 3 robot missions: parking → production station
    parking = factory.get_stations_by_type(StationType.PARKING_BAY)
    targets = [
        factory.get_stations_by_type(StationType.CELL_ASSEMBLY)[0],      # amr_1 → cell assembly
        factory.get_stations_by_type(StationType.MODULE_PACKING)[0],     # amr_2 → module packing
        factory.get_stations_by_type(StationType.TESTING_QC)[1],         # amr_3 → testing/QC
    ]

    robot_configs = [
        {"id": "AMR-1", "start": parking[0], "goal": targets[0], "color": "#2E86AB"},
        {"id": "AMR-2", "start": parking[2], "goal": targets[1], "color": "#E94F37"},
        {"id": "AMR-3", "start": parking[4], "goal": targets[2], "color": "#44BBA4"},
    ]

    # Plan Dubins paths for each robot
    planned_paths = {}
    sampled_paths = {}
    print("  Planning Dubins paths...")
    for cfg in robot_configs:
        sp = cfg["start"].position
        gp = cfg["goal"].approach_point
        sh = cfg["start"].orientation
        gh = cfg["goal"].approach_heading

        path = dubins.plan((sp[0], sp[1], sh), (gp[0], gp[1], gh))
        if path:
            wps = dubins.sample_path(path, (sp[0], sp[1], sh), step_size=0.5)
            xy = [(w[0], w[1]) for w in wps]
            planned_paths[cfg["id"]] = xy
            sampled_paths[cfg["id"]] = wps  # includes heading
            print(f"    {cfg['id']}: {path.path_type.value}, {path.total_length:.1f}m, {len(wps)} waypoints")
        else:
            # Fallback: straight line
            planned_paths[cfg["id"]] = [sp, gp]
            sampled_paths[cfg["id"]] = [(sp[0], sp[1], sh), (gp[0], gp[1], gh)]
            print(f"    {cfg['id']}: straight fallback")

    # Generate frames
    frames = []
    print(f"  Rendering {n_frames} frames...")

    for frame_idx in range(n_frames):
        progress = frame_idx / max(1, n_frames - 1)  # 0.0 → 1.0

        robots = []
        trail_history = {}

        for cfg in robot_configs:
            rid = cfg["id"]
            wps = sampled_paths[rid]
            n_wps = len(wps)

            # Robot-specific progress (stagger: AMR-2 starts 10% later, AMR-3 20% later)
            offset = {"AMR-1": 0.0, "AMR-2": 0.10, "AMR-3": 0.20}.get(rid, 0.0)
            local_progress = max(0.0, min(1.0, (progress - offset) / (1.0 - offset)))

            wp_idx = int(local_progress * (n_wps - 1))
            wp_idx = min(wp_idx, n_wps - 1)

            x, y, heading = wps[wp_idx]

            state = "navigating"
            if local_progress >= 0.95:
                state = "docking"
            elif local_progress <= 0.0:
                state = "parked"

            robots.append({
                "id": rid, "x": x, "y": y, "heading": heading,
                "color": cfg["color"], "state": state,
            })

            # Trail: all waypoints up to current position
            trail_history[rid] = [(w[0], w[1]) for w in wps[:wp_idx + 1]]

        status = f"Time: {progress * 60:.0f}s / 60s\n"
        for r in robots:
            status += f"{r['id']}: {r['state']}\n"

        img = _render_frame(
            factory, robots,
            planned_paths=planned_paths,
            trail_history=trail_history,
            status_text=status.strip(),
            title="Multi-AMR Factory Navigation — Fleet Simulation",
        )
        frames.append(img)

        if (frame_idx + 1) % 15 == 0:
            print(f"    Frame {frame_idx + 1}/{n_frames}")

    # Save GIF
    print(f"  Saving GIF to {output_path}...")
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=150,  # ms per frame
        loop=0,
    )

    # File size
    size_kb = Path(output_path).stat().st_size / 1024
    print(f"  Done! {len(frames)} frames, {size_kb:.0f} KB")
    return output_path


# ---------------------------------------------------------------------------
# GIF 2: Obstacle Avoidance — AMR detects person, stops, alerts, resumes
# ---------------------------------------------------------------------------

def create_obstacle_avoidance_gif(output_path: str, n_frames: int = 80) -> str:
    """Create animated GIF showing obstacle detection and avoidance.

    Scenario:
      1. AMR navigates toward a station via Dubins path (frames 0-25)
      2. A person appears in the path (frame 26)
      3. AMR detects obstacle, begins slowing (frames 26-35)
      4. AMR performs emergency stop, sends alert to hub (frames 36-50)
      5. Person clears the path (frame 51)
      6. AMR receives clearance, resumes navigation (frames 52-80)

    Args:
        output_path: File path for the output GIF.
        n_frames: Number of animation frames.

    Returns:
        Path to the created GIF.
    """
    print("  Creating factory environment...")
    factory = FactoryEnvironment()
    dubins = DubinsPlanner(turning_radius=3.0)

    # Robot mission: parking bay 1 → cell assembly 2
    parking = factory.get_stations_by_type(StationType.PARKING_BAY)
    target = factory.get_stations_by_type(StationType.MODULE_PACKING)[1]

    start_pos = parking[1].position
    start_heading = parking[1].orientation
    goal_pos = target.approach_point
    goal_heading = target.approach_heading

    # Plan path
    path = dubins.plan(
        (start_pos[0], start_pos[1], start_heading),
        (goal_pos[0], goal_pos[1], goal_heading),
    )
    if path:
        wps = dubins.sample_path(path, (start_pos[0], start_pos[1], start_heading), step_size=0.4)
        print(f"  Path: {path.path_type.value}, {path.total_length:.1f}m, {len(wps)} waypoints")
    else:
        print("  WARNING: No Dubins path found, using straight line")
        wps = [
            (start_pos[0], start_pos[1], start_heading),
            (goal_pos[0], goal_pos[1], goal_heading),
        ]

    full_path = [(w[0], w[1]) for w in wps]
    n_wps = len(wps)

    # Obstacle appears at ~35% of path
    obstacle_wp_idx = int(n_wps * 0.35)
    obs_x, obs_y = wps[obstacle_wp_idx][0], wps[obstacle_wp_idx][1]
    # Offset slightly to the side of the path
    obs_x += 1.0
    obs_y += 0.5

    # Phase boundaries (in frame numbers)
    phase_obstacle_appears = int(n_frames * 0.30)
    phase_emergency_stop = int(n_frames * 0.42)
    phase_obstacle_clears = int(n_frames * 0.62)
    phase_resume = int(n_frames * 0.67)

    frames = []
    print(f"  Rendering {n_frames} frames...")

    stopped_wp_idx = None  # The waypoint where robot stopped

    for frame_idx in range(n_frames):
        progress = frame_idx / max(1, n_frames - 1)
        obstacles = []
        alert_text = None
        state = "navigating"

        # Determine robot position based on phase
        if frame_idx < phase_obstacle_appears:
            # Phase 1: Normal navigation
            local_p = frame_idx / phase_obstacle_appears * 0.33
            wp_idx = int(local_p * (n_wps - 1))
            state = "navigating"

        elif frame_idx < phase_emergency_stop:
            # Phase 2: Obstacle detected, slowing down
            # Robot was at ~33% of path, now creeping toward obstacle
            approach_progress = (frame_idx - phase_obstacle_appears) / (phase_emergency_stop - phase_obstacle_appears)
            wp_idx = int((0.33 + approach_progress * 0.02) * (n_wps - 1))  # barely moves
            obstacles = [{"x": obs_x, "y": obs_y, "radius": 2.0, "label": "PERSON DETECTED"}]
            alert_text = (
                "ALERT: Obstacle detected!\n"
                f"Type: Person\n"
                f"Distance: {max(0.5, 3.0 - approach_progress * 2.5):.1f}m\n"
                f"Action: SLOWING DOWN"
            )
            state = "navigating"

        elif frame_idx < phase_obstacle_clears:
            # Phase 3: Emergency stop — robot halted
            wp_idx = int(0.35 * (n_wps - 1))  # stopped before obstacle
            stopped_wp_idx = wp_idx
            obstacles = [{"x": obs_x, "y": obs_y, "radius": 2.0, "label": "PERSON DETECTED"}]
            alert_text = (
                "EMERGENCY STOP!\n"
                "Alert sent to Fleet Hub\n"
                f"Robot: AMR-1\n"
                f"Location: ({obs_x:.0f}, {obs_y:.0f})\n"
                "Waiting for path clear..."
            )
            state = "emergency_stop"

        elif frame_idx < phase_resume:
            # Phase 4: Obstacle clearing
            wp_idx = stopped_wp_idx or int(0.35 * (n_wps - 1))
            # Obstacle fading out
            fade = 1.0 - (frame_idx - phase_obstacle_clears) / (phase_resume - phase_obstacle_clears)
            if fade > 0.3:
                obstacles = [{"x": obs_x, "y": obs_y, "radius": 2.0 * fade, "label": "Clearing..."}]
            alert_text = (
                "Obstacle clearing...\n"
                "Scanning environment\n"
                "Preparing to resume"
            )
            state = "waiting"

        else:
            # Phase 5: Resume navigation
            resume_start_wp = stopped_wp_idx or int(0.35 * (n_wps - 1))
            resume_progress = (frame_idx - phase_resume) / (n_frames - 1 - phase_resume)
            remaining_wps = n_wps - 1 - resume_start_wp
            wp_idx = resume_start_wp + int(resume_progress * remaining_wps)
            alert_text = (
                "PATH CLEAR\n"
                "Hub: Resume navigation\n"
                f"Progress: {min(100, int(wp_idx / (n_wps-1) * 100))}%"
            )
            state = "navigating"

        wp_idx = max(0, min(wp_idx, n_wps - 1))
        x, y, heading = wps[wp_idx]

        # Build robot data
        robots = [{
            "id": "AMR-1", "x": x, "y": y, "heading": heading,
            "color": "#E94F37" if state == "emergency_stop" else "#2E86AB",
            "state": state,
        }]

        # Trail
        trail = {
            "AMR-1": [(w[0], w[1]) for w in wps[:wp_idx + 1]],
        }

        # Status box
        time_s = frame_idx * 0.5  # ~0.5s per frame
        status = f"Time: {time_s:.1f}s\nState: {state.upper()}"
        if state == "emergency_stop":
            status += "\nVelocity: 0.0 m/s"
        elif state == "navigating":
            status += "\nVelocity: 1.2 m/s"
        elif state == "waiting":
            status += "\nVelocity: 0.0 m/s"

        img = _render_frame(
            factory, robots,
            planned_paths={"AMR-1": full_path},
            trail_history=trail,
            obstacles=obstacles,
            alert_text=alert_text,
            status_text=status,
            title="Obstacle Avoidance — Emergency Stop & Hub Alert",
        )
        frames.append(img)

        if (frame_idx + 1) % 20 == 0:
            print(f"    Frame {frame_idx + 1}/{n_frames}")

    # Save GIF
    print(f"  Saving GIF to {output_path}...")
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=200,  # ms per frame — slower for obstacle scenario
        loop=0,
    )

    size_kb = Path(output_path).stat().st_size / 1024
    print(f"  Done! {len(frames)} frames, {size_kb:.0f} KB")
    return output_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    """Generate both animated GIFs."""
    output_dir = Path(__file__).resolve().parent.parent.parent / "docs" / "images"
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("Multi-AMR Factory Navigation — GIF Generator")
    print("=" * 60)

    print("\n[1/2] Fleet Navigation GIF")
    fleet_gif = create_fleet_navigation_gif(
        str(output_dir / "fleet_navigation.gif"),
        n_frames=60,
    )

    print(f"\n[2/2] Obstacle Avoidance GIF")
    obstacle_gif = create_obstacle_avoidance_gif(
        str(output_dir / "obstacle_avoidance.gif"),
        n_frames=80,
    )

    print("\n" + "=" * 60)
    print("All GIFs created successfully!")
    print(f"  Fleet:    {fleet_gif}")
    print(f"  Obstacle: {obstacle_gif}")
    print("=" * 60)


if __name__ == "__main__":
    main()
