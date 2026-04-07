# Main Demo Script Guide

## Overview

`main.py` is the entry point for demonstrating the multi-AMR factory navigation system. It provides 6 comprehensive demos showcasing different capabilities of the system.

## Location

```
/sessions/magical-epic-bell/mnt/Job Profile built-up/ros2-multi-amr-factory/main.py
```

## Quick Start

```bash
# Run all demos
python3 main.py

# Run a specific demo
python3 main.py --demo factory
python3 main.py --demo full

# Customize full simulation
python3 main.py --demo full --num-robots 12 --num-tasks 20 --steps 1000
```

## Available Demos

### Demo 1: Factory Layout Visualization
**What it shows:** The complete factory floor with all stations, aisles, intersections, and restricted zones.

**Key output:**
- Station counts by type
- Factory dimensions and grid resolution
- Visual map with color-coded stations
- Approach points and orientation markers
- Aisle and intersection locations

**Command:**
```bash
python3 main.py --demo factory
```

### Demo 2: AMR Robot Model & Kinematics
**What it shows:** Single robot movement with circular trajectory, turret rotation, and battery discharge.

**Key features:**
- Differential drive kinematics
- Independent turret rotation (0-360°)
- Battery discharge modeling
- Trajectory visualization
- Realistic physics constraints

**Command:**
```bash
python3 main.py --demo amr
```

### Demo 3: Dubins Path Planning
**What it shows:** Optimal curved path planning between multiple pose pairs using Dubins curves.

**Key features:**
- All 6 Dubins path types (LSL, LSR, RSL, RSR, RLR, LRL)
- Spline smoothing of waypoints
- Path length statistics
- Multi-robot trajectory visualization
- Turning radius constraints

**Command:**
```bash
python3 main.py --demo dubins
```

### Demo 4: Docking Sequence
**What it shows:** Complete docking interaction with a charging station.

**Key features:**
- Approach phase
- Alignment phase
- Final positioning
- Charging state
- Battery management

**Command:**
```bash
python3 main.py --demo docking
```

### Demo 5: Traffic Management at Intersections
**What it shows:** Multi-robot coordination with traffic priority and right-of-way control.

**Key features:**
- 3 robots with crossing paths
- Priority-based yielding (HIGH, NORMAL, LOW)
- Intersection claim/release
- Deadlock detection
- Velocity modifiers based on traffic state

**Command:**
```bash
python3 main.py --demo traffic
```

### Demo 6: Full Fleet Simulation (THE BIG ONE)
**What it shows:** Complete end-to-end simulation with 8 robots, 15 production tasks, and 500 steps.

**Key features:**
- 8 robot fleet initialization
- Automatic task generation following production flow
- Real-time fleet coordination
- Battery-aware path planning
- Snapshot captures at key moments (steps 0, 100, 250, 500)
- Production metrics (distance, battery, task counts)
- Per-robot status reporting

**Command:**
```bash
python3 main.py --demo full
# Or with custom parameters:
python3 main.py --demo full --num-robots 12 --num-tasks 20 --steps 1000
```

## Command-Line Arguments

```
--demo {factory,amr,dubins,docking,traffic,full,all}
    Which demo(s) to run (default: all)

--num-robots NUM_ROBOTS
    Number of robots for full simulation (default: 8)

--num-tasks NUM_TASKS
    Number of production tasks (default: 15)

--steps STEPS
    Simulation steps for full demo (default: 500)
```

## Output Format

Each demo prints:
1. **Header** — Demo name and separator
2. **Configuration** — Parameters and setup info
3. **Progress** — Key milestones and calculations
4. **Visualization** — Matplotlib figures (interactive)
5. **Statistics** — Final metrics and results

## Example Output

```
======================================================================
DEMO 1: Factory Layout Visualization
======================================================================
Factory dimensions: 100.0m x 80.0m
Grid resolution: 0.5m
Total stations: 37

Stations by type:
  - incoming_material: 4
  - cell_assembly: 6
  - module_packing: 4
  - pack_integration: 3
  - testing_qc: 4
  - shipping: 3
  - charging_station: 4
  - parking_bay: 6

Generating factory floor visualization...

Factory layout demo completed successfully!
```

## Architecture

The script uses the following components:

| Module | Purpose |
|--------|---------|
| `src/factory/environment.py` | Factory layout and obstacles |
| `src/amr/robot.py` | Robot physics and kinematics |
| `src/planning/dubins.py` | Curved path planning |
| `src/planning/spline.py` | Path smoothing |
| `src/planning/path_planner.py` | High-level path planning |
| `src/docking/dock_controller.py` | Docking sequences |
| `src/traffic/traffic_manager.py` | Intersection management |
| `src/fleet/coordinator.py` | Fleet orchestration |
| `src/utils/dashboard.py` | Visualization |

## Error Handling

The script is defensive about:
- Missing modules or methods
- Import errors
- Invalid parameters
- Exception handling with try-except blocks

If a component is unavailable, the demo prints a helpful message and continues.

## Performance Notes

- **Factory layout**: <1 second
- **AMR model**: ~5 seconds (100 physics steps)
- **Dubins paths**: ~2 seconds (3 paths)
- **Docking sequence**: ~10 seconds
- **Traffic management**: ~15 seconds
- **Full simulation**: ~30-60 seconds (500 steps with 8 robots)

Total runtime for all demos: ~2-3 minutes

## Dependencies

- Python 3.6+
- numpy
- matplotlib
- math (standard library)

## Portfolio Use

This demo script is excellent for:
- **GitHub portfolio** — Shows complete system capability
- **Technical interviews** — Demonstrates robotics knowledge
- **Documentation** — Entry point for understanding the codebase
- **Verification** — Ensures all systems are working together

## Author

Muskaan Maheshwari

## License

MIT
