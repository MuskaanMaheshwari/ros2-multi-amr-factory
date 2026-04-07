# ROS2 Integration & Deployment

**Author:** Muskaan Maheshwari

## Overview

This document covers practical deployment, Gazebo integration, and advanced configuration of the multi-AMR factory system. For ROS2 node architecture and message formats, see the root-level `ROS2_ARCHITECTURE.md`.

## Gazebo Integration (Conceptual)

The factory environment can be visualized in Gazebo with realistic robot physics and sensor simulation. Currently the system uses Python-based physics; future work includes real-time Gazebo integration.

### Planned Gazebo Workflow

```bash
# Terminal 1: Start Gazebo with factory world
roslaunch gazebo_ros empty_world.launch world_name:=$(ros2 pkg prefix ros2_multi_amr_factory)/worlds/factory.world

# Terminal 2: Spawn robots at specific positions
for i in {1..5}; do
  ros2 run gazebo_ros spawn_entity.py \
    -entity amr_00$i \
    -file $(ros2 pkg prefix ros2_multi_amr_factory)/urdf/amr.urdf \
    -x $((i * 5)) -y 0.0 -z 0.1
done

# Terminal 3: Launch factory fleet controller
ros2 launch ros2_nodes factory_fleet.launch.py num_robots:=5
```

### Robot URDF Structure

The URDF defines the robot morphology for collision checking and visualization:

```xml
<robot name="amr">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1.2 0.8 0.4"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="1.2" radius="0.6"/>  <!-- safety circle -->
      </geometry>
    </collision>
  </link>

  <link name="turret_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.3"/>
      </geometry>
    </visual>
  </link>

  <joint name="turret_rotation" type="revolute">
    <parent link="base_link"/>
    <child link="turret_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="6.28" effort="10" velocity="1.0"/>
  </joint>
</robot>
```

### RViz Configuration

The included RViz config (`config/factory_viz.rviz`) provides a factory-specific dashboard:

**Displays**:
- Robot footprints and safety circles
- Odometry frames and TF tree
- Robot velocity vectors (cmd_vel subscription)
- Planned paths from path_planner
- Station locations (docks, chargers, assembly)
- Intersection zones colored by state (FREE/CLAIMED/OCCUPIED)

**Plugins**:
- Fleet status panel (robot count, utilization, task queue)
- Task browser (pending, active, completed)
- Robot diagnostics viewer

**Running RViz with the factory**:

```bash
# Launch with RViz enabled
ros2 launch ros2_nodes factory_fleet.launch.py num_robots:=3 enable_rviz:=true

# Or launch separately
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix ros2_multi_amr_factory)/config/factory_viz.rviz
```

## Performance Tuning

### Update Rates

The default configuration uses conservative rates suitable for Python-based simulation. For real-time deployment or when integrating with actual hardware, adjust the `update_rate` parameter.

| Component | Default | Recommended (Hardware) | Notes |
|-----------|---------|------------------------|-------|
| Fleet Manager | 10 Hz | 20 Hz | Task assignment is rate-limited to 0.5 Hz internally |
| Traffic Controller | 10 Hz | 20–50 Hz | Deadlock detection runs every 2 seconds regardless |
| Path Planner | On-demand | On-demand | Typical latency 20–50 ms per plan |
| AMR Driver | 10 Hz | 20–50 Hz | Odometry and status publication |

**Example: High-speed deployment**

```bash
ros2 launch ros2_nodes factory_fleet.launch.py \
    num_robots:=10 \
    update_rate:=50
```

### CPU & Memory Usage (Estimated)

**Single robot driver**: ~5–10% CPU, ~50 MB RAM (trajectory history + state cache)

**Fleet manager (5 robots)**: ~8–15% CPU, ~30 MB RAM (task queues, robot state)

**Traffic controller**: ~2–5% CPU, ~10 MB RAM (intersection graph)

**Path planner**: ~0–20% CPU (depending on planning frequency), ~20 MB RAM

**Total system (5 robots)**: ~300 MB RAM, ~30% CPU on modern hardware

### Battery Model Configuration

The battery discharge model can be tuned per-robot. Edit the fleet manager config YAML:

```yaml
fleet_manager:
  ros__parameters:
    robot_specs:
      - robot_id: amr_001
        battery_capacity_ah: 100.0
        battery_voltage_v: 48.0
        discharge_rate_ah_per_meter: 0.15  # Ah consumed per meter traveled
        charge_rate_ah_per_hour: 50.0  # Charge speed (hours to full)
```

The default discharge model:
- 48V, 100 Ah (4.8 kWh) per robot
- Discharge: ~0.15 Ah per meter (15 km range with max payload)
- Charging: 50 Ah/hour (45 minutes from 20% to 100%)

## Task Injection and External Interfaces

### Adding Tasks via ROS2 Topic

External systems (MES, SCADA, upper-level fleet scheduler) can add tasks by publishing to `/fleet/add_task`:

```bash
# Single task: assemble cells
ros2 topic pub --once /fleet/add_task std_msgs/msg/String \
  "{data: '{\"pickup_station\": \"MATERIAL_IN_1\", \"dropoff_station\": \"CELL_ASSEMBLY_1\", \"priority\": 1}'}"

# Continuous task injection (e.g., every 2 seconds)
ros2 topic pub --rate 0.5 /fleet/add_task std_msgs/msg/String \
  "{data: '{\"pickup_station\": \"CELL_ASSEMBLY_1\", \"dropoff_station\": \"MODULE_PACKING_1\", \"priority\": 2}'}"
```

**Task JSON Schema**:

```json
{
  "pickup_station": "string",      // Station name from factory layout
  "dropoff_station": "string",     // Destination station name
  "payload_kg": 100.0,             // Optional, default 0
  "priority": 1,                   // 0=critical, 1=high, 2=normal, 3=low
  "deadline_seconds": null         // Optional, for deadline-aware scheduling
}
```

### Task Lifecycle Events

The fleet manager publishes task completion events:

```bash
ros2 topic echo /fleet/task_completed
```

Output:

```json
{
  "timestamp": 1234567890.123,
  "task_id": "TASK_0001",
  "robot_id": "amr_001",
  "pickup_station": "MATERIAL_IN_1",
  "dropoff_station": "CELL_ASSEMBLY_1",
  "status": "completed",
  "duration_seconds": 125.5,
  "distance_meters": 45.2,
  "energy_consumed_wh": 85.0
}
```

## Multi-Floor / Multi-Zone Scaling

The current implementation models a single factory floor. Extending to multiple floors requires:

1. **Z-axis in coordinates**: Modify `FactoryEnvironment` to include elevation
2. **Elevator queue system**: Add elevator nodes with queue management
3. **Per-floor traffic control**: Separate `TrafficController` instances per floor
4. **3D path planning**: Extend `FactoryPathPlanner` with vertical waypoints
5. **Battery model updates**: Account for elevation changes in discharge rate

**Rough architecture for multi-floor**:

```
fleet_manager (central)
├── floor_1_traffic_controller
├── floor_2_traffic_controller
├── elevator_manager
├── amr_driver_001 (floor 1)
├── amr_driver_002 (floor 1)
├── amr_driver_003 (floor 2)
└── path_planner_3d
```

## Troubleshooting

### Nodes Exit Immediately

**Symptom**: Nodes enter Active state but crash after a few seconds.

**Diagnosis**: Check node logs and `colcon build` output.

```bash
# Rebuild with verbose output
colcon build --packages-select ros2_multi_amr_factory --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Run node directly to see exception
python3 -m ros2_nodes.amr_driver --ros-args --params-file config/amr_driver.yaml
```

**Common causes**:
- Missing Python dependencies (numpy, scipy)
- Malformed YAML config
- ROS2 package not sourced

### Deadlock Detected But Never Resolved

**Symptom**: `/traffic/deadlock_alert` publishes repeatedly, robots stuck at intersection.

**Causes**:
- Insufficient robot count relative to task load
- Path planner returning identical paths for all robots
- Traffic controller parameters too conservative

**Resolution**:
- Increase `num_robots` or reduce task injection rate
- Tuning: lower `safety_radius_m` from 1.5 to 1.0, increase `deadlock_timeout_s` to 10.0
- Check path planner is finding diverse routes via `/path_planner/plan_stats`

### Tasks Complete Very Slowly

**Symptom**: Task duration 5+ minutes for short distances.

**Diagnosis**:
```bash
# Monitor task queue
ros2 topic echo /fleet/status | grep pending_tasks

# Check robot states
ros2 topic echo /amr_001/status
```

**Causes**:
- Battery-triggered charging cycles (robots pause to charge frequently)
- Low fleet utilization (few robots, many tasks)
- Traffic congestion at popular intersections

**Mitigation**:
- Reduce battery discharge model (lower `discharge_rate_ah_per_meter`)
- Add more robots to fleet
- Rebalance station placement to reduce intersection congestion

## Known Limitations

1. **Python-only physics**: Not real-time compatible; simulation runs at ~10 Hz
2. **Perfect odometry**: Assumes error-free pose estimates (no SLAM needed)
3. **Static factory layout**: Obstacles and stations cannot move during operation
4. **No sensor simulation**: LiDAR, camera, proximity sensors not modeled
5. **Simplified docking**: 5-second fixed duration regardless of alignment quality

## Future Enhancements

1. **Gazebo real-time physics**: Migrate from Python to Gazebo Sim (C++) for real-time operation
2. **Nav2 stack integration**: Replace custom path planner with Nav2 for SLAM support
3. **Heterogeneous fleets**: Support different robot types with different specs
4. **Machine learning**: Learned task assignment policies to replace heuristics
5. **Multi-factory coordination**: Coordinate across multiple factory floors or locations

## References

- ROS2 Humble docs: https://docs.ros.org/en/humble/
- Gazebo Sim: https://gazebosim.org/
- Nav2 documentation: https://docs.nav2.org/
- Geometry messages: http://docs.ros.org/en/humble/p/geometry_msgs/
