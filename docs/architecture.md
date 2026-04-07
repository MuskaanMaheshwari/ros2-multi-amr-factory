# System Architecture

**Author:** Muskaan Maheshwari

## Overview

The ros2-multi-amr-factory is a modular, physics-based simulation system for autonomous mobile robot (AMR) coordination in battery manufacturing environments. The system combines realistic robot dynamics, fleet-level task orchestration, traffic management, and path planning into an integrated framework.

## Core Components

### 1. Robot Physics Engine (`src/amr/robot.py`)

**Module:** `AMRRobot`, `AMRState`, `BatteryModel`

Provides physics-based simulation of individual TK-AMR Automake style robots with:

- **Differential Drive Kinematics**: Supports linear and angular velocity control with realistic acceleration/deceleration limits
- **Independent Turret**: Separate rotation capability for load alignment during docking
- **Battery Management**: Realistic 48V lithium battery model with discharge modeling and charging
- **State Machine**: 8 discrete states (IDLE, NAVIGATING, DOCKING, LOADING, UNLOADING, CHARGING, PARKED, EMERGENCY_STOP, WAITING)
- **Collision Detection**: Footprint corners and safety circle for boundary checking
- **Trajectory Recording**: Complete movement history for analysis

**Key Attributes:**
- Max payload: 500 kg
- Max linear speed: 1.5 m/s
- Max angular speed: 2.0 rad/s
- Max turret speed: 1.0 rad/s
- Dimensions: 1.2m (length) x 0.8m (width) x 0.4m (height)
- Battery capacity: 100 Ah at 48V (4.8 kWh)

### 2. Fleet Coordinator (`src/fleet/coordinator.py`)

**Module:** `FleetCoordinator`, `Task`, `TaskType`, `TaskPriority`

Central orchestration system managing the entire robot fleet with:

- **Task Management**: Priority queue-based task assignment with 4 priority levels
- **Robot Assignment**: Heuristic-based allocation considering distance, battery, and robot state
- **Production Flow Tracking**: Follows realistic battery manufacturing pipeline
- **Docking Sequences**: 5-second simulated docking/loading/unloading cycles
- **Battery-Aware Planning**: Automatic charging task creation for low-battery robots
- **Deadlock Detection**: Simple proximity-based deadlock identification
- **Performance Metrics**: Throughput, utilization, task completion tracking

**Production Flow:**
```
INCOMING_MATERIAL → CELL_ASSEMBLY → MODULE_PACKING →
PACK_INTEGRATION → TESTING_QC → SHIPPING
```

### 3. Traffic Manager (`src/traffic/traffic_manager.py`)

**Module:** `TrafficManager`, `Intersection`, `IntersectionState`, `DeadlockInfo`

Manages multi-robot traffic flows to prevent collisions:

- **Intersection Control**: FREE, CLAIMED, OCCUPIED states for intersection zones
- **Priority-Based Right-of-Way**: 4-level priority system (EMERGENCY, HIGH, NORMAL, LOW)
- **Reservation System**: Robots reserve intersections before entering
- **Queue Management**: Waiting queues at congested intersections
- **Deadlock Detection**: Monitors and logs deadlock situations
- **Deadlock Resolution**: Attempts rerouting or yielding strategies

**Intersection Defaults:**
- Zone radius: 3.0 meters
- Safety separation: 1.5 meters between robots

### 4. Path Planning (`src/planning/path_planner.py`)

**Module:** `FactoryPathPlanner`, `FactoryEnvironment`, `DubinsPlanner`, `CubicSplineSmoother`

Multi-method path planning combining graph search with curve generation:

- **A* Graph Search**: Finds optimal paths through aisle waypoint network
- **Dubins Curves**: Generates smooth, kinematically feasible paths with bounded curvature
- **Cubic Spline Smoothing**: Refines waypoints into smooth curves
- **Collision Checking**: Validates paths against obstacles
- **Statistics Tracking**: Measures path length, computation time, curvature

### 5. Factory Environment (`src/factory/environment.py`)

**Module:** `FactoryEnvironment`, `Station`, `StationType`, `FactoryZone`

Models the physical factory layout with:

- **Station Types**: 9 types (incoming material, assembly, packing, integration, QC, shipping, charging, parking, docks)
- **Zone Types**: 5 types (production, aisle, intersection, restricted, buffer)
- **Approach Points**: Predefined entry positions for station docking
- **Obstacle Model**: Grid-based obstacle representation
- **Visualization**: Matplotlib-based floor plan rendering

**Factory Dimensions:**
- Width: 100 meters
- Height: 80 meters
- Grid resolution: 0.5 meters per cell

### 6. Docking Controller (`src/docking/dock_controller.py`)

**Module:** `DockController`, `DockingSequence`, `DockState`

Manages precise robot-to-station docking sequences:

- **Alignment**: Rotates turret to match station orientation
- **Approach**: Fine-grained positional adjustment
- **Contact**: Validates physical contact with dock
- **Load Transfer**: Simulates payload exchange (5-second cycle)
- **Timeout Handling**: Recovers from docking failures

## Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Main Simulation Loop                        │
│                     (main.py, ~10 Hz)                           │
└────────────────────┬────────────────────────────────────────────┘
                     │
        ┌────────────┴────────────┬──────────────────┐
        │                         │                  │
        ▼                         ▼                  ▼
┌──────────────────┐    ┌──────────────────┐  ┌──────────────────┐
│  Fleet           │    │  Traffic         │  │  Path            │
│  Coordinator     │    │  Manager         │  │  Planner         │
│                  │    │                  │  │                  │
│ • Task assign    │    │ • Intersection   │  │ • Route comp.    │
│ • Robot state    │    │   state update   │  │ • Obstacle check │
│ • Docking ops    │    │ • Deadlock det.  │  │ • Curve smooth   │
│ • Battery mgmt   │    │ • Priority mgmt  │  │ • Dubins curves  │
└────────┬─────────┘    └────────┬─────────┘  └────────┬─────────┘
         │                       │                     │
         └───────────────────────┼─────────────────────┘
                                 │
                    ┌────────────┴────────────┐
                    │                         │
                    ▼                         ▼
          ┌──────────────────┐      ┌──────────────────┐
          │  AMR Physics     │      │  Factory         │
          │  (per robot)     │      │  Environment     │
          │                  │      │                  │
          │ • Kinematics     │      │ • Station locs   │
          │ • Battery        │      │ • Obstacles      │
          │ • Turret control │      │ • Zone types     │
          │ • Collisions     │      │ • Visualization  │
          └────────┬─────────┘      └──────────────────┘
                   │
                   ▼
          ┌──────────────────┐
          │  Visualization   │
          │  & Logging       │
          │                  │
          │ • Trajectory     │
          │ • Status updates │
          │ • Performance    │
          └──────────────────┘
```

## Node Communication Graph (ROS2 Integration)

```
                        /fleet/status ──────────┐
                                                 │
    /fleet/assign_task ──────┐                  │
                             ▼                  ▼
                    ┌─────────────────────┐
                    │  fleet_manager_node │
                    │  (central control)  │
                    └─────────────────────┘
                             │
                ┌────────────┬──────────────┐
                │            │              │
                ▼            ▼              ▼
         ┌──────────┐ ┌──────────┐  ┌──────────────┐
         │path_     │ │traffic_  │  │amr_driver_   │
         │planner_  │ │controller│  │node (×N)     │
         │node      │ │_node     │  │              │
         └──────────┘ └──────────┘  └──────────────┘
              │            │              │
              │            │         ┌────┴────────┐
              │            │         │             │
              │    /traffic/intersect/amr_001/odom
              │    ion_states         /amr_001/battery
              │                       /amr_001/cmd_vel
              │                       /amr_001/turret_cmd
              │
         /path_planner/
         last_plan

Topics Flow:
- Publishers: odom, battery, turret_state, status
- Subscribers: cmd_vel, turret_cmd
- Services: emergency_stop, reset, plan_path
```

## Simulation Pipeline

### 1. Initialization Phase

```python
# Create factory environment
factory_env = FactoryEnvironment(...)

# Create robot fleet
robots = create_default_fleet(
    n_robots=5,
    parking_positions=factory_env.parking_bays
)

# Initialize coordinators
fleet_coord = FleetCoordinator(
    factory_env, robots, path_planner,
    traffic_manager, dock_controller
)
```

### 2. Main Loop Iteration (dt = 0.1 seconds)

```
Step 1: Fleet Coordinator Update
  ├─ Assign pending tasks to available robots
  ├─ Execute task state machines
  │  └─ For each robot: NAVIGATING → DOCKING → LOADING/UNLOADING → idle
  ├─ Check for deadlocks
  └─ Compile fleet statistics

Step 2: Path Planning (on-demand)
  ├─ For each navigating robot
  ├─ Plan path to next waypoint
  ├─ Update local path plan
  └─ Request traffic clearance

Step 3: Traffic Management Update
  ├─ Update intersection states
  ├─ Process reservation queue
  ├─ Apply velocity modifiers based on congestion
  └─ Log deadlock attempts

Step 4: Robot Physics Update (per robot)
  ├─ Apply velocity commands
  ├─ Update pose (kinematics)
  ├─ Rotate turret
  ├─ Discharge battery based on motion
  ├─ Record trajectory
  └─ Check for collisions

Step 5: Visualization
  ├─ Update robot positions on canvas
  ├─ Render trajectories
  ├─ Display fleet status
  └─ Log performance metrics
```

### 3. Task Execution Flow

```
Production Task Lifecycle:
  pending
    ↓ (robot assigned)
  assigned
    ↓ (navigate to pickup)
  in_progress (navigation phase)
    ↓ (reached pickup)
  pickup
    ↓ (docking complete)
  transport
    ↓ (navigate to dropoff)
  dropoff (at destination)
    ↓ (docking complete)
  completed → statistics
    or
  failed → error handling

Charging Task Lifecycle:
  pending
    ↓
  assigned
    ↓ (navigate to charger)
  in_progress
    ↓ (arrived)
  pickup (charging active)
    ↓ (battery >= target)
  completed → back to idle
```

## Module Interactions

### Fleet Coordinator ↔ Path Planner

- **Input**: Robot position, target station
- **Output**: Waypoint sequence, path length
- **Frequency**: Once per task assignment (0.5 Hz average)

### Fleet Coordinator ↔ Traffic Manager

- **Input**: Robot positions, planned waypoints
- **Output**: Intersection reservations, velocity modifiers
- **Frequency**: Every update (10 Hz)

### Fleet Coordinator ↔ Dock Controller

- **Input**: Robot ID, station ID, task type
- **Output**: Docking completion status
- **Frequency**: During docking (5-second sequences)

### Traffic Manager ↔ Path Planner

- **Input**: Congestion zones, blocked intersections
- **Output**: Reroute request (indirect)
- **Frequency**: As needed (~1 Hz)

### Path Planner ↔ Factory Environment

- **Input**: Aisle waypoints, obstacle grid
- **Output**: None (read-only)
- **Frequency**: Continuous

## State Management

### Robot State Transitions

```
       ┌─────────────┐
       │    IDLE     │◄────────────────────┐
       └──────┬──────┘                     │
              │ (task assigned)           │ (charging complete)
              ▼                           │
       ┌─────────────┐    ┌───────────┐  │
       │ NAVIGATING  ├───►│ DOCKING   │──┤
       └─────────────┘    └───────────┘  │
                             │           │
                             ├─ LOADING  │
                             │           │
                             └─ UNLOADING

       ┌─────────────┐
       │  CHARGING   │
       └─────────────┘

Emergency transitions:
       Any state ───(emergency_stop)──► EMERGENCY_STOP
```

### Task State Transitions

```
pending ──(assign)──► assigned ──(start)──► in_progress
                                     │
                                  (pickup, transport, dropoff)
                                     │
                                     ▼
                                  completed
                                  (or)
                                  failed
```

## Performance Characteristics

### Computational Complexity

- **Fleet Coordinator**: O(n) task assignment, O(n²) deadlock check
  - n = number of robots
  - Execution: ~5-10 ms per cycle

- **Traffic Manager**: O(m) intersection update
  - m = number of intersections
  - Execution: ~1-2 ms per cycle

- **Path Planner**: O(w² log w) A* search
  - w = aisle waypoints
  - Execution: ~20-50 ms per plan (typical 10m path)

### Scalability

- **Tested Configuration**: 10 robots, 30 tasks/minute
- **Limitation**: Python-based simulation (not real-time)
- **ROS2 Integration**: Enables parallel processing across cores

## Extension Points

1. **Custom Planners**: Replace `FactoryPathPlanner` with RRT*, PRM, or other algorithms
2. **Traffic Policies**: Modify intersection priority rules in `TrafficManager`
3. **Battery Models**: Extend `BatteryModel` for different chemistries/configurations
4. **Robot Types**: Support heterogeneous fleet with different specs in `AMRSpecs`
5. **Obstacle Handling**: Integrate SLAM or sensor fusion for dynamic obstacles
6. **Gazebo Integration**: Real-time visualization with physics engine

## References

- ROS2 Node Architecture: Standard Python node structure
- Dubins Paths: Optimal length curves with bounded curvature
- A* Algorithm: Graph search with heuristic cost estimation
- Differential Drive: Standard non-holonomic mobile robot kinematics
- Traffic Flow Models: Adapted from vehicular traffic engineering
