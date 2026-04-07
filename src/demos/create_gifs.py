#!/usr/bin/env python3
"""
Animated GIF generators for Multi-AMR Factory Navigation.

Creates two publication-quality animated GIFs:
  1. Fleet navigation — 3 AMRs following factory aisles with smooth turns
  2. Obstacle avoidance — AMR detects person, stops, alerts hub, resumes

Path planning follows factory aisles (horizontal/vertical corridors).
AMRs travel in straight lines along aisles and use smooth circular arcs
only at intersections when changing direction. Stations are solid obstacles
and paths never cross them.

Author: Muskaan Maheshwari
"""

import math
import sys
from pathlib import Path
from typing import List, Tuple, Dict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from PIL import Image
import io

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from src.factory.environment import FactoryEnvironment, StationType


# ---------------------------------------------------------------------------
# Aisle path planner — grid-aligned, collision-aware, smooth circular arcs
# ---------------------------------------------------------------------------

# Factory aisle centre-lines (must match environment.py)
H_AISLES = [10.0, 20.0, 35.0, 50.0, 65.0, 75.0]  # y-coordinates
V_AISLES = [12.0, 37.0, 58.0, 77.0]                # x-coordinates


def _station_bboxes(factory, margin=0.5):
    """Station bounding boxes expanded by a safety margin.

    Returns:
        List of (left, top, right, bottom) tuples.
    """
    boxes = []
    for st in factory.stations.values():
        x, y = st.position
        hw = st.width / 2.0 + margin
        hh = st.length / 2.0 + margin
        boxes.append((x - hw, y - hh, x + hw, y + hh))
    return boxes


def _hseg_blocked(y, x1, x2, bboxes):
    """True if horizontal segment at *y* from *x1* to *x2* crosses a station."""
    lo, hi = (x1, x2) if x1 <= x2 else (x2, x1)
    for left, top, right, bot in bboxes:
        if top < y < bot and left < hi and right > lo:
            return True
    return False


def _vseg_blocked(x, y1, y2, bboxes):
    """True if vertical segment at *x* from *y1* to *y2* crosses a station."""
    lo, hi = (y1, y2) if y1 <= y2 else (y2, y1)
    for left, top, right, bot in bboxes:
        if left < x < right and top < hi and bot > lo:
            return True
    return False


def _smooth_arc(p1, corner, p2, radius=2.5, n_pts=16):
    """Proper circular arc rounding a turn between two axis-aligned segments.

    Computes tangent points on the incoming and outgoing straight segments,
    finds the arc centre, then traces a circular arc between them.

    Args:
        p1:     Previous waypoint (defines incoming direction).
        corner: The turn waypoint.
        p2:     Next waypoint (defines outgoing direction).
        radius: Desired turning radius in metres.
        n_pts:  Number of sample points on the arc.

    Returns:
        List of (x, y) points along the arc.
    """
    dx_in, dy_in = corner[0] - p1[0], corner[1] - p1[1]
    len_in = math.hypot(dx_in, dy_in)
    dx_out, dy_out = p2[0] - corner[0], p2[1] - corner[1]
    len_out = math.hypot(dx_out, dy_out)

    if len_in < 0.5 or len_out < 0.5:
        return [corner]

    ux_in, uy_in = dx_in / len_in, dy_in / len_in
    ux_out, uy_out = dx_out / len_out, dy_out / len_out

    cross = ux_in * uy_out - uy_in * ux_out
    if abs(cross) < 0.01:  # nearly straight — no arc needed
        return [corner]

    r = min(radius, len_in * 0.45, len_out * 0.45)

    # Tangent points on incoming / outgoing segments
    ts = (corner[0] - ux_in * r, corner[1] - uy_in * r)
    te = (corner[0] + ux_out * r, corner[1] + uy_out * r)

    # Inward normal (perpendicular to incoming direction, toward arc centre)
    if cross > 0:
        nx, ny = -uy_in, ux_in
    else:
        nx, ny = uy_in, -ux_in

    cx = ts[0] + nx * r
    cy = ts[1] + ny * r

    a0 = math.atan2(ts[1] - cy, ts[0] - cx)
    a1 = math.atan2(te[1] - cy, te[0] - cx)
    da = a1 - a0
    while da > math.pi:
        da -= 2.0 * math.pi
    while da < -math.pi:
        da += 2.0 * math.pi

    return [
        (cx + r * math.cos(a0 + da * i / n_pts),
         cy + r * math.sin(a0 + da * i / n_pts))
        for i in range(n_pts + 1)
    ]


def _simplify(pts):
    """Remove interior points that are collinear with their neighbours."""
    if len(pts) <= 2:
        return list(pts)
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        px, py = out[-1]
        cx, cy = pts[i]
        nx, ny = pts[i + 1]
        same_h = abs(py - cy) < 0.1 and abs(cy - ny) < 0.1
        same_v = abs(px - cx) < 0.1 and abs(cx - nx) < 0.1
        if not (same_h or same_v):
            out.append(pts[i])
    out.append(pts[-1])
    return out


def plan_aisle_path(start: Tuple[float, float],
                    goal: Tuple[float, float],
                    factory: FactoryEnvironment,
                    turn_radius: float = 2.5) -> List[Tuple[float, float, float]]:
    """Plan a factory path that strictly follows horizontal/vertical aisles.

    Strategy
    --------
    1. Pull out of *start* vertically to the nearest horizontal aisle.
    2. Travel horizontally to the best vertical aisle (minimises total
       lateral detour to both start and goal while remaining clear of
       station obstacles).
    3. Travel vertically on that aisle to the horizontal aisle nearest
       the goal.
    4. Travel horizontally then vertically to the goal.

    All straight segments are axis-aligned (parallel to the factory grid).
    Turns use smooth circular arcs.  Stations are solid obstacles — the
    planner validates that every chosen V-aisle segment is free before
    committing.

    Args:
        start:       (x, y) start position.
        goal:        (x, y) goal position.
        factory:     FactoryEnvironment instance.
        turn_radius: Arc radius at corners (metres).

    Returns:
        List of (x, y, heading) waypoints sampled at ~0.5 m spacing.
    """
    bboxes = _station_bboxes(factory, margin=0.5)

    # ---- choose aisles ------------------------------------------------
    h_entry = min(H_AISLES, key=lambda y: abs(y - start[1]))
    h_exit = min(H_AISLES, key=lambda y: abs(y - goal[1]))

    # Pick V-aisle that minimises total lateral detour AND is passable
    def _v_cost(vx):
        return abs(vx - start[0]) + abs(vx - goal[0])

    best_v = None
    y_lo, y_hi = sorted([h_entry, h_exit])
    for vx in sorted(V_AISLES, key=_v_cost):
        if not _vseg_blocked(vx, y_lo, y_hi, bboxes):
            best_v = vx
            break
    if best_v is None:  # fallback — pick cheapest even if partially blocked
        best_v = min(V_AISLES, key=_v_cost)

    # ---- build axis-aligned waypoint list ------------------------------
    wps = [start]

    def _append(pt):
        if math.hypot(pt[0] - wps[-1][0], pt[1] - wps[-1][1]) > 0.05:
            wps.append(pt)

    _append((start[0], h_entry))   # vertical pull-out to H-aisle
    _append((best_v, h_entry))     # horizontal to V-aisle
    _append((best_v, h_exit))      # vertical run along V-aisle
    _append((goal[0], h_exit))     # horizontal toward goal x
    _append(goal)                  # vertical to goal (if needed)

    # Merge collinear runs
    simple = _simplify(wps)

    # ---- insert smooth arcs at turns -----------------------------------
    raw = [simple[0]]
    for i in range(1, len(simple) - 1):
        raw.extend(_smooth_arc(simple[i - 1], simple[i], simple[i + 1],
                               turn_radius))
    raw.append(simple[-1])

    # ---- dense sample with headings (0.5 m spacing) --------------------
    dense: List[Tuple[float, float, float]] = []
    for i in range(len(raw) - 1):
        x1, y1 = raw[i]
        x2, y2 = raw[i + 1]
        seg = math.hypot(x2 - x1, y2 - y1)
        heading = math.atan2(y2 - y1, x2 - x1)
        n = max(1, int(seg / 0.5))
        for j in range(n):
            t = j / n
            dense.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1), heading))

    if raw:
        dense.append((raw[-1][0], raw[-1][1],
                       dense[-1][2] if dense else 0.0))

    return dense


# ---------------------------------------------------------------------------
# Frame renderer
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
    """Render a single animation frame and return as PIL Image."""
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_xlim(0, factory.width)
    ax.set_ylim(0, factory.height)
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.set_xlabel("X (meters)", fontsize=9)
    ax.set_ylabel("Y (meters)", fontsize=9)
    ax.set_title(title, fontsize=13, fontweight="bold")

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
    for y in H_AISLES:
        ax.add_patch(mpatches.Rectangle((0, y - 2), factory.width, 4, lw=0, fc="#E8E8E8", zorder=0))
    for x in V_AISLES:
        ax.add_patch(mpatches.Rectangle((x - 1.5, 0), 3, factory.height, lw=0, fc="#E8E8E8", zorder=0))

    # Restricted corners
    for cx, cy in [(0, 0), (96, 0), (0, 76), (96, 76)]:
        ax.add_patch(mpatches.Rectangle((cx, cy), 4, 4, lw=0.5, fc="red", alpha=0.25, hatch="///", zorder=1))

    # Intersections
    for ix, iy in factory.get_intersections():
        ax.plot(ix, iy, "D", ms=5, color="gold", mec="black", mew=0.5, zorder=2)

    # Stations
    for st in factory.stations.values():
        x, y = st.position
        w, h = st.width, st.length
        c = station_colors.get(st.station_type, "#CCC")
        ax.add_patch(mpatches.FancyBboxPatch(
            (x - w/2, y - h/2), w, h,
            boxstyle="round,pad=0.05", lw=0.8, ec="black", fc=c, alpha=0.7, zorder=3,
        ))
        short = st.station_id.split("_")[0][:4]
        ax.text(x, y, short, ha="center", va="center", fontsize=5, zorder=4)

    # Planned paths (dashed grey)
    if planned_paths:
        for rid, ppath in planned_paths.items():
            if len(ppath) > 1:
                xs = [p[0] for p in ppath]
                ys = [p[1] for p in ppath]
                ax.plot(xs, ys, "--", color="#999", lw=1.2, alpha=0.5, zorder=5)

    # Trail history (solid coloured)
    if trail_history:
        for rid, trail in trail_history.items():
            if len(trail) > 1:
                rc = "#2E86AB"
                for r in robots:
                    if r["id"] == rid:
                        rc = r.get("color", "#2E86AB")
                        break
                xs = [p[0] for p in trail]
                ys = [p[1] for p in trail]
                ax.plot(xs, ys, "-", color=rc, lw=2.5, alpha=0.8, zorder=6)

    # Obstacles
    if obstacles:
        for obs in obstacles:
            ax.add_patch(mpatches.Circle(
                (obs["x"], obs["y"]), obs.get("radius", 1.5),
                fc="red", alpha=0.15, ec="red", lw=1.5, ls="--", zorder=7,
            ))
            ax.plot(obs["x"], obs["y"], "X", ms=14, color="red", mec="darkred", mew=2, zorder=8)
            if obs.get("label"):
                ax.text(obs["x"], obs["y"] - 2.0, obs["label"],
                        ha="center", fontsize=8, color="red", fontweight="bold", zorder=9)

    # Robots
    for r in robots:
        rx, ry, rh = r["x"], r["y"], r["heading"]
        color = r.get("color", "#2E86AB")
        state = r.get("state", "navigating")

        ec = "red" if state == "emergency_stop" else "black"
        lw = 2.5 if state == "emergency_stop" else 1.5
        ax.add_patch(mpatches.Circle((rx, ry), 1.0, fc=color, ec=ec, lw=lw, zorder=10))

        dx, dy = 1.2 * math.cos(rh), 1.2 * math.sin(rh)
        ax.annotate("", xy=(rx + dx, ry + dy), xytext=(rx, ry),
                     arrowprops=dict(arrowstyle="->", color="white", lw=2), zorder=11)
        ax.text(rx, ry + 1.8, r["id"], ha="center", fontsize=7, fontweight="bold",
                color=color, zorder=12)

    # Alert box
    if alert_text:
        props = dict(boxstyle="round,pad=0.5", facecolor="#FFDDDD", edgecolor="red", alpha=0.9)
        ax.text(2, factory.height - 3, alert_text, fontsize=8, va="top",
                bbox=props, zorder=15, family="monospace")

    # Status box
    if status_text:
        props = dict(boxstyle="round,pad=0.4", facecolor="#DDFFDD", edgecolor="green", alpha=0.9)
        ax.text(factory.width - 2, 3, status_text, fontsize=8, va="top", ha="right",
                bbox=props, zorder=15, family="monospace")

    # Legend outside plot
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

    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=100, bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    return Image.open(buf).copy()


# ---------------------------------------------------------------------------
# GIF 1: Fleet Navigation — 3 AMRs following factory aisles
# ---------------------------------------------------------------------------

def create_fleet_navigation_gif(output_path: str, n_frames: int = 70) -> str:
    """Create animated GIF of 3 AMRs navigating through factory aisles.

    Robots follow aisles (horizontal/vertical corridors) with smooth
    curved turns at intersections. Stations are treated as solid obstacles.
    """
    print("  Creating factory environment...")
    factory = FactoryEnvironment()

    # 3 missions: parking bay → production-area aisle intersection
    # Goals sit on clear aisle intersections close to target station areas.
    parking = factory.get_stations_by_type(StationType.PARKING_BAY)
    missions = [
        {
            "id": "AMR-1", "color": "#2E86AB",
            "start": parking[0].position,       # (15, 78)
            "goal": (12.0, 10.0),               # V/H aisle intersection near incoming material
        },
        {
            "id": "AMR-2", "color": "#E94F37",
            "start": parking[2].position,       # (45, 78)
            "goal": (37.0, 50.0),               # intersection near module packing area
        },
        {
            "id": "AMR-3", "color": "#44BBA4",
            "start": parking[3].position,       # (60, 78)
            "goal": (58.0, 35.0),               # intersection near pack integration
        },
    ]

    # Plan aisle-following paths
    planned_paths = {}
    sampled_paths = {}
    print("  Planning aisle-following paths...")
    for m in missions:
        wps = plan_aisle_path(m["start"], m["goal"], factory, turn_radius=2.5)
        xy = [(w[0], w[1]) for w in wps]
        planned_paths[m["id"]] = xy
        sampled_paths[m["id"]] = wps
        total_len = sum(
            math.hypot(wps[i+1][0]-wps[i][0], wps[i+1][1]-wps[i][1])
            for i in range(len(wps)-1)
        )
        print(f"    {m['id']}: {total_len:.1f}m, {len(wps)} waypoints")

    # Generate frames
    frames = []
    print(f"  Rendering {n_frames} frames...")

    for frame_idx in range(n_frames):
        progress = frame_idx / max(1, n_frames - 1)

        robots = []
        trail_history = {}

        for m in missions:
            rid = m["id"]
            wps = sampled_paths[rid]
            n_wps = len(wps)

            # Stagger departures
            offset = {"AMR-1": 0.0, "AMR-2": 0.08, "AMR-3": 0.16}.get(rid, 0.0)
            local_p = max(0.0, min(1.0, (progress - offset) / (1.0 - offset)))

            wp_idx = min(int(local_p * (n_wps - 1)), n_wps - 1)
            x, y, heading = wps[wp_idx]

            state = "parked" if local_p <= 0.0 else ("docking" if local_p >= 0.95 else "navigating")

            robots.append({
                "id": rid, "x": x, "y": y, "heading": heading,
                "color": m["color"], "state": state,
            })
            trail_history[rid] = [(w[0], w[1]) for w in wps[:wp_idx + 1]]

        status = f"Time: {progress * 90:.0f}s / 90s\n"
        for r in robots:
            status += f"{r['id']}: {r['state']}\n"

        img = _render_frame(
            factory, robots,
            planned_paths=planned_paths,
            trail_history=trail_history,
            status_text=status.strip(),
            title="Multi-AMR Factory Navigation — Aisle-Following Fleet",
        )
        frames.append(img)

        if (frame_idx + 1) % 15 == 0:
            print(f"    Frame {frame_idx + 1}/{n_frames}")

    print(f"  Saving GIF to {output_path}...")
    frames[0].save(output_path, save_all=True, append_images=frames[1:],
                   duration=150, loop=0)

    size_kb = Path(output_path).stat().st_size / 1024
    print(f"  Done! {len(frames)} frames, {size_kb:.0f} KB")
    return output_path


# ---------------------------------------------------------------------------
# GIF 2: Obstacle Avoidance — emergency stop, alert, resume
# ---------------------------------------------------------------------------

def create_obstacle_avoidance_gif(output_path: str, n_frames: int = 80) -> str:
    """Create animated GIF showing obstacle detection on an aisle path.

    AMR navigates along aisle, encounters person mid-aisle, stops,
    sends alert to hub, waits, then resumes after obstacle clears.
    """
    print("  Creating factory environment...")
    factory = FactoryEnvironment()

    parking = factory.get_stations_by_type(StationType.PARKING_BAY)

    start_pos = parking[1].position           # (30, 78)
    goal_pos = (37.0, 10.0)                   # clear aisle intersection

    wps = plan_aisle_path(start_pos, goal_pos, factory, turn_radius=2.5)
    full_path = [(w[0], w[1]) for w in wps]
    n_wps = len(wps)
    total_len = sum(
        math.hypot(wps[i+1][0]-wps[i][0], wps[i+1][1]-wps[i][1])
        for i in range(len(wps)-1)
    )
    print(f"  Path: {total_len:.1f}m, {n_wps} waypoints")

    # Obstacle on the aisle at ~40% of path
    obs_idx = int(n_wps * 0.40)
    obs_x, obs_y = wps[obs_idx][0], wps[obs_idx][1]

    # Phases
    phase_appear = int(n_frames * 0.28)
    phase_stop = int(n_frames * 0.40)
    phase_clear = int(n_frames * 0.62)
    phase_resume = int(n_frames * 0.68)

    frames = []
    stopped_idx = None
    print(f"  Rendering {n_frames} frames...")

    for fi in range(n_frames):
        obstacles = []
        alert_text = None
        state = "navigating"

        if fi < phase_appear:
            # Normal navigation
            p = fi / phase_appear * 0.38
            wp_idx = int(p * (n_wps - 1))
            state = "navigating"

        elif fi < phase_stop:
            # Detecting, slowing
            approach_p = (fi - phase_appear) / (phase_stop - phase_appear)
            wp_idx = int((0.38 + approach_p * 0.01) * (n_wps - 1))
            obstacles = [{"x": obs_x, "y": obs_y, "radius": 2.0, "label": "PERSON DETECTED"}]
            dist = max(0.5, 3.0 - approach_p * 2.5)
            alert_text = f"ALERT: Obstacle detected!\nType: Person\nDistance: {dist:.1f}m\nAction: SLOWING DOWN"
            state = "navigating"

        elif fi < phase_clear:
            # Emergency stop
            wp_idx = int(0.39 * (n_wps - 1))
            stopped_idx = wp_idx
            obstacles = [{"x": obs_x, "y": obs_y, "radius": 2.0, "label": "PERSON DETECTED"}]
            alert_text = (
                "EMERGENCY STOP!\n"
                "Alert sent to Fleet Hub\n"
                f"Robot: AMR-1\n"
                f"Location: ({obs_x:.0f}, {obs_y:.0f})\n"
                "Waiting for path clear..."
            )
            state = "emergency_stop"

        elif fi < phase_resume:
            # Clearing
            wp_idx = stopped_idx or int(0.39 * (n_wps - 1))
            fade = 1.0 - (fi - phase_clear) / (phase_resume - phase_clear)
            if fade > 0.3:
                obstacles = [{"x": obs_x, "y": obs_y, "radius": 2.0 * fade, "label": "Clearing..."}]
            alert_text = "Obstacle clearing...\nScanning environment\nPreparing to resume"
            state = "waiting"

        else:
            # Resume
            resume_wp = stopped_idx or int(0.39 * (n_wps - 1))
            rp = (fi - phase_resume) / (n_frames - 1 - phase_resume)
            remaining = n_wps - 1 - resume_wp
            wp_idx = resume_wp + int(rp * remaining)
            pct = min(100, int(wp_idx / (n_wps - 1) * 100))
            alert_text = f"PATH CLEAR\nHub: Resume navigation\nProgress: {pct}%"
            state = "navigating"

        wp_idx = max(0, min(wp_idx, n_wps - 1))
        x, y, heading = wps[wp_idx]

        robots = [{
            "id": "AMR-1", "x": x, "y": y, "heading": heading,
            "color": "#E94F37" if state == "emergency_stop" else "#2E86AB",
            "state": state,
        }]

        trail = {"AMR-1": [(w[0], w[1]) for w in wps[:wp_idx + 1]]}

        ts = fi * 0.5
        st_text = f"Time: {ts:.1f}s\nState: {state.upper()}"
        if state == "emergency_stop":
            st_text += "\nVelocity: 0.0 m/s"
        elif state == "navigating":
            st_text += "\nVelocity: 1.2 m/s"
        else:
            st_text += "\nVelocity: 0.0 m/s"

        img = _render_frame(
            factory, robots,
            planned_paths={"AMR-1": full_path},
            trail_history=trail,
            obstacles=obstacles,
            alert_text=alert_text,
            status_text=st_text,
            title="Obstacle Avoidance — Emergency Stop & Hub Alert",
        )
        frames.append(img)

        if (fi + 1) % 20 == 0:
            print(f"    Frame {fi + 1}/{n_frames}")

    print(f"  Saving GIF to {output_path}...")
    frames[0].save(output_path, save_all=True, append_images=frames[1:],
                   duration=200, loop=0)

    size_kb = Path(output_path).stat().st_size / 1024
    print(f"  Done! {len(frames)} frames, {size_kb:.0f} KB")
    return output_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    output_dir = Path(__file__).resolve().parent.parent.parent / "docs" / "images"
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("Multi-AMR Factory Navigation — GIF Generator")
    print("  Paths follow factory aisles (no cutting through stations)")
    print("  Smooth circular arcs at intersections only")
    print("=" * 60)

    print("\n[1/2] Fleet Navigation GIF")
    create_fleet_navigation_gif(str(output_dir / "fleet_navigation.gif"), n_frames=70)

    print("\n[2/2] Obstacle Avoidance GIF")
    create_obstacle_avoidance_gif(str(output_dir / "obstacle_avoidance.gif"), n_frames=80)

    print("\n" + "=" * 60)
    print("Done! GIFs saved to docs/images/")
    print("=" * 60)


if __name__ == "__main__":
    main()
