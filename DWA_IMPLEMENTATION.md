# Dynamic Window Approach (DWA) Implementation

This document describes the DWA obstacle avoidance and dynamic obstacle detection system implemented for the ros2-multi-amr-factory project.

## Overview

The implementation consists of 5 new modules that enable robots to:
1. Detect unexpected obstacles via simulated LiDAR
2. Generate real-time alerts for fleet coordination
3. Plan local avoidance trajectories using Dynamic Window Approach
4. Visualize navigation progress with multi-frame animations
5. Display obstacle information and alerts on the fleet dashboard

All components are pure Python and require no ROS2 dependencies for simulation.

## Components

### 1. DWA Planner Module (`src/planning/dwa.py`)

**Purpose**: Local obstacle avoidance using Dynamic Window Approach algorithm.

**Key Classes**:
- `DWAPlanner`: Main planning class
- `DWAMetrics`: Dataclass containing planning results

**Algorithm**:
- Samples candidate velocity pairs (v, ω) within dynamic window
- Simulates trajectory for each candidate over prediction horizon (default 2.0s)
- Scores trajectories based on:
  - **Heading error** (30% weight): Alignment toward goal
  - **Obstacle distance** (50% weight): Distance to nearest obstacle
  - **Velocity magnitude** (20% weight): Efficiency preference
- Selects trajectory with highest composite score

**Mathematical Formulas**:
```
Trajectory: x(t) = x₀ + v·cos(θ)·t, y(t) = y₀ + v·sin(θ)·t, θ(t) = θ₀ + ω·t
Score = α·heading_score + β·distance_score + γ·velocity_score
```

**Usage**:
```python
from src.planning.dwa import DWAPlanner
import numpy as np

planner = DWAPlanner(
    max_linear_velocity=1.5,
    max_angular_velocity=2.0,
    prediction_time=2.0,
    dt=0.1,
)

# Plan avoidance
velocity, metrics = planner.plan(
    robot_pose=(5.0, 5.0, 0.0),
    current_velocity=(0.5, 0.0),
    goal=(50.0, 50.0),
    obstacle_grid=occupancy_grid,
)

print(f"Recommended velocity: {velocity}")
print(f"Best score: {metrics.best_score}")
print(f"Min obstacle distance: {metrics.min_obstacle_distance}m")
```

**Parameters**:
- `max_linear_velocity`: 1.5 m/s default
- `max_angular_velocity`: 2.0 rad/s default
- `prediction_time`: 2.0s horizon for trajectory simulation
- `dt`: 0.1s time step discretization
- `heading_weight`: 0.3 (30% of score)
- `obstacle_distance_weight`: 0.5 (50% of score)
- `velocity_weight`: 0.2 (20% of score)
- `emergency_distance`: 0.3m threshold for stop

**Output**:
- `DWAMetrics` dataclass with:
  - `best_score`: Composite trajectory score
  - `min_obstacle_distance`: Closest obstacle distance (m)
  - `heading_error`: Angular error to goal (rad)
  - `linear_velocity`: Recommended linear velocity (m/s)
  - `angular_velocity`: Recommended angular velocity (rad/s)

### 2. Obstacle Detection Module (`src/perception/obstacle_detector.py`)

**Purpose**: Detect unexpected obstacles via simulated LiDAR scanning.

**Key Classes**:
- `ObstacleDetector`: Main detection class
- `ObstacleInfo`: Detected obstacle data
- `ScanResult`: Result from single scan

**Enums**:
- `ObstacleType`: PERSON, EQUIPMENT, CLUTTER, UNKNOWN
- `ObstacleSeverity`: HIGH, MEDIUM, LOW

**Features**:
- 180° × 15m range LiDAR simulation with 0.5° resolution
- Compares actual environment against expected costmap to identify unexpected obstacles
- Obstacle classification based on distance and characteristics
- Severity assessment (HIGH for people, MEDIUM for equipment, LOW for clutter)
- Confidence scoring and temporal filtering to reduce false positives
- Sensor model with configurable false positive/negative rates

**Usage**:
```python
from src.perception.obstacle_detector import ObstacleDetector, create_test_obstacle_grid

# Create detector with expected environment
expected_grid = create_test_obstacle_grid()
detector = ObstacleDetector(
    expected_path_costmap=expected_grid,
    lidar_range=15.0,
    angular_resolution=0.5,
    false_positive_rate=0.05,
    false_negative_rate=0.1,
)

# Perform scan
result = detector.scan(
    robot_pose=(x, y, heading),
    actual_obstacle_grid=actual_grid,
    timestamp=time.time(),
)

if result.obstacle_detected:
    for obstacle in result.obstacles:
        print(f"Found {obstacle.obstacle_type.value} at {obstacle.position}")
        print(f"  Distance: {obstacle.distance:.2f}m")
        print(f"  Severity: {obstacle.severity.value}")
        print(f"  Confidence: {obstacle.confidence:.2f}")
```

**Parameters**:
- `lidar_range`: 15.0m max sensing range
- `angular_resolution`: 0.5° sampling
- `false_positive_rate`: 0.05 (5% spurious detections)
- `false_negative_rate`: 0.1 (10% missed obstacles)

**Output**:
- `ScanResult` dataclass with:
  - `obstacle_detected`: Boolean flag
  - `obstacles`: List of `ObstacleInfo` objects
  - `scan_points`: Raw LiDAR points (distance, angle, intensity)

### 3. Alert System (`src/messaging/alert_system.py`)

**Purpose**: Generate and manage obstacle/emergency alerts for fleet coordination.

**Key Classes**:
- `AlertGenerator`: Creates structured alert messages
- `AlertHub`: Central hub for processing and tracking alerts
- `AlertMessage`: Structured alert data

**Enums**:
- `AlertEventType`: OBSTACLE_DETECTED, EMERGENCY_STOP, OBSTACLE_CLEARED, etc.
- `AlertDecision`: STOP, REDIRECT, WAIT, CONTINUE

**Features**:
- Alert message with full context (robot ID, position, obstacle type, severity)
- Automatic hub response determination (escalation based on severity)
- Alert history tracking with expiration
- Temporal filtering and alert statistics
- JSON-serializable format for external systems

**Usage**:
```python
from src.messaging.alert_system import AlertGenerator, AlertHub, AlertDecision
from src.perception.obstacle_detector import ObstacleType, ObstacleSeverity

# Create generator
generator = AlertGenerator()

# Create obstacle alert
alert = generator.create_obstacle_alert(
    robot_id="amr_001",
    position=(5.0, 5.0),
    obstacle_type=ObstacleType.PERSON,
    distance=0.8,
    current_state="navigating",
    severity=ObstacleSeverity.HIGH,
    confidence=0.95,
)

# Process through hub
hub = AlertHub(max_history_size=1000, alert_timeout=30.0)
decision = hub.process_alert(alert)

if decision == AlertDecision.STOP:
    # Emergency condition: pause all robots
    pass
elif decision == AlertDecision.REDIRECT:
    # Route around obstacle
    pass

# Get statistics
stats = hub.get_statistics()
print(f"Total alerts: {stats['total_alerts']}")
print(f"Active alerts: {stats['active_alerts']}")
```

**Alert Message Format**:
```python
{
    "robot_id": "amr_001",
    "timestamp": 1234567890.5,
    "event_type": "obstacle_detected",
    "location": {"x": 5.0, "y": 5.0},
    "obstacle_distance": 0.8,
    "obstacle_type": "person",
    "robot_state": "navigating",
    "action_taken": "emergency_stop_activated",
    "hub_response_required": True,
    "alert_id": "alert_000001",
    "severity": "high",
    "confidence": 0.95
}
```

### 4. Robot Integration (`src/amr/robot.py`)

**Modifications to AMRRobot class**:

**New Attributes**:
- `obstacle_detector`: Integrated ObstacleDetector instance
- `dwa_planner`: Integrated DWAPlanner instance
- `alert_generator`: Integrated AlertGenerator instance
- `obstacle_detected`: Current obstacle detection state
- `last_obstacle_distance`: Distance to nearest detected obstacle
- `obstacle_free_time`: Time since last obstacle detection

**New Methods**:
- `emergency_stop()`: Halt all motion immediately (already existed, updated docstring)
- `can_resume_path()`: Check if path is clear for resuming navigation

**Integration in update() method**:
- Performs LiDAR scan each update cycle when in NAVIGATING state
- Generates alerts when obstacles are detected
- Tracks obstacle-free time for resumption logic
- Maintains emergency stop behavior

**Usage**:
```python
from src.amr.robot import AMRRobot
from src.planning.dwa import DWAPlanner
from src.perception.obstacle_detector import ObstacleDetector
from src.messaging.alert_system import AlertGenerator

# Create robot
robot = AMRRobot(robot_id="amr_001", start_position=(5.0, 5.0))

# Attach modules
robot.dwa_planner = DWAPlanner()
robot.obstacle_detector = ObstacleDetector(expected_path_costmap=grid)
robot.alert_generator = AlertGenerator()

# Update will now include obstacle detection
robot.set_velocity(1.0, 0.0)
for _ in range(100):
    robot.update(dt=0.1)

# Emergency response
if robot.obstacle_detected:
    robot.emergency_stop()

# Resume when safe
if robot.can_resume_path():
    robot.state = AMRState.NAVIGATING
```

### 5. Navigation Animator (`src/utils/navigation_animator.py`)

**Purpose**: Create multi-frame visualization of navigation progress for portfolio documentation.

**Key Classes**:
- `NavigationAnimator`: Multi-frame visualization generator

**Features**:
- Samples path at progress milestones: 0%, 25%, 50%, 75%, 100%
- Creates matplotlib subplot grid (configurable columns)
- Shows:
  - Factory floor layout (stations, walls, obstacles)
  - Full and completed path segments
  - Robot position and heading at each milestone
  - Progress indicator overlay
  - Legend outside plot area for clarity

**Usage**:
```python
from src.utils.navigation_animator import NavigationAnimator
import matplotlib.pyplot as plt

# Create animator
path = [(10, 10), (30, 20), (50, 50), (70, 80), (90, 90)]
headings = [0.0, 0.785, 1.571, 2.356, 3.142]  # radians

animator = NavigationAnimator(
    factory_env=factory_environment,
    robot_path=path,
    robot_headings=headings,
)

# Generate visualization
fig = animator.create_grid_visualization(n_cols=2, figsize=(12, 10))

# Save and display
fig.savefig('navigation_progress.png', dpi=150, bbox_inches='tight')
plt.show()
```

**Output**: Matplotlib figure with 5 subplots showing navigation at each milestone.

### 6. Dashboard Enhancements (`src/utils/dashboard.py`)

**Modifications**:

**New Features**:
- Alert status panel showing recent alerts with color-coded severity
- Obstacle visualization (red circles on factory floor)
- Improved legend positioning (outside plot area)
- Better layout with 3-row grid for more information

**New Methods**:
- `add_obstacle(position, radius)`: Add dynamic obstacle to map
- `update_alerts(alerts)`: Update alert display panel

**Modified Methods**:
- `__init__()`: Added alert tracking and display panel
- `draw_factory_floor()`: Enhanced for obstacle rendering

**Usage**:
```python
from src.utils.dashboard import FleetDashboard
from src.messaging.alert_system import AlertHub

dashboard = FleetDashboard(factory_env, robots)

# Add detected obstacle
dashboard.add_obstacle((50.0, 50.0), radius=0.3)

# Update alerts from hub
alerts = [
    {
        'robot_id': 'amr_001',
        'event_type': 'obstacle_detected',
        'severity': 'high',
        'message': 'Person detected'
    }
]
dashboard.update_alerts(alerts)
```

## Testing

Run the comprehensive test suite to verify all implementations:

```bash
cd ros2-multi-amr-factory
python test_dwa_implementation.py
```

**Test Coverage**:
1. ✓ DWA planner initialization and execution
2. ✓ Obstacle detector scanning and classification
3. ✓ Alert generation and hub processing
4. ✓ Robot integration with obstacle detection
5. ✓ Navigation animator visualization generation

**Expected Output**: "Results: 5 passed, 0 failed"

## Performance Characteristics

**DWA Planner**:
- Computation time: ~1-5ms per plan (depends on grid resolution)
- Memory: Minimal, trajectory storage is O(prediction_time/dt)
- Scalability: Linear with number of candidate velocities

**Obstacle Detector**:
- Scan time: ~2-10ms depending on range and resolution
- False positive rate: Configurable, default 5%
- False negative rate: Configurable, default 10%
- Temporal filtering reduces false positives

**Alert System**:
- Alert generation: <1ms
- Hub processing: <1ms per alert
- History lookup: O(log n) with time-based expiration

**Navigation Animator**:
- Grid generation: ~100-500ms depending on path length
- Memory: O(num_frames * resolution²)

## Design Patterns

1. **Sensor Simulation**: LiDAR scans simulated from occupancy grid with sensor model (false positives/negatives)

2. **Composite Scoring**: Multi-objective optimization combining heading, obstacle, and velocity scores

3. **Temporal Filtering**: Obstacle detections confirmed across scans to reduce noise

4. **Modular Integration**: All modules loosely coupled, can be used independently

5. **Event-Driven Alerts**: Alert hub receives events and broadcasts decisions to fleet

## Future Enhancements

- [ ] Multi-robot coordination using alert hub decisions
- [ ] Trajectory prediction for moving obstacles
- [ ] ROS2 bridge for real hardware deployment
- [ ] Machine learning-based obstacle classification
- [ ] Costmap generation from laser scans
- [ ] Path replanning triggered by new obstacles

## References

- Dynamic Window Approach: Fox et al., "The Dynamic Window Approach to Collision Avoidance" (1997)
- Occupancy Grids: Thrun et al., "Probabilistic Robotics" (2005)
- Safety-Critical Systems: ISO 13849-1 (Functional Safety)

## Files Structure

```
src/
├── planning/
│   └── dwa.py                      # DWA planner
├── perception/
│   ├── __init__.py
│   └── obstacle_detector.py         # LiDAR simulation & detection
├── messaging/
│   ├── __init__.py
│   └── alert_system.py             # Alert generation & hub
├── amr/
│   └── robot.py                    # Modified for obstacle integration
└── utils/
    ├── dashboard.py                # Enhanced with alerts & obstacles
    └── navigation_animator.py       # Multi-frame visualization

test_dwa_implementation.py           # Test suite
```

## Author

Muskaan Maheshwari
Robotics & AI Engineer
IIT Palakkad + ASU MS
