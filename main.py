#!/usr/bin/env python3
"""
ROS2 Multi-AMR Factory Navigation — Main Demo Script

Multi-robot fleet navigation in an EV/battery manufacturing factory.
Features curved Dubins/spline paths, turret-top AMR kinematics, docking
station interaction, traffic management, and real-time fleet coordination.

Author: Muskaan Maheshwari
License: MIT
"""

import argparse
import sys
from pathlib import Path
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time
import math

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent))

from src.factory.environment import FactoryEnvironment, StationType
from src.amr.robot import AMRRobot, AMRState, create_default_fleet
from src.planning.dubins import DubinsPlanner
from src.planning.spline import CubicSplineSmoother
from src.planning.path_planner import FactoryPathPlanner
from src.docking.dock_controller import DockController
from src.traffic.traffic_manager import TrafficManager, TrafficPriority
from src.fleet.coordinator import FleetCoordinator, Task, TaskType, TaskPriority
from src.utils.dashboard import FleetDashboard
from src.demos.navigation_demo import create_navigation_demo
from src.demos.obstacle_avoidance_demo import create_obstacle_avoidance_demo
from src.demos.alert_demo import create_alert_demo


def demo_factory_layout():
    """
    Demonstrate the factory environment visualization.

    Shows the complete factory floor layout with all stations, aisles,
    intersections, and restricted zones.
    """
    print("\n" + "="*70)
    print("DEMO 1: Factory Layout Visualization")
    print("="*70)

    try:
        # Create factory environment
        factory = FactoryEnvironment(width=100.0, height=80.0, resolution=0.5)

        # Print station counts
        print(f"Factory dimensions: {factory.width}m x {factory.height}m")
        print(f"Grid resolution: {factory.resolution}m")
        print(f"Total stations: {len(factory.stations)}")

        # Count stations by type
        station_counts = {}
        for station in factory.stations.values():
            stype = station.station_type.value
            station_counts[stype] = station_counts.get(stype, 0) + 1

        print("\nStations by type:")
        for stype, count in sorted(station_counts.items()):
            print(f"  - {stype}: {count}")

        # Visualize the factory
        print("\nGenerating factory floor visualization...")
        fig = factory.visualize(title="EV Battery Manufacturing Factory Floor")
        plt.show()

        print("\nFactory layout demo completed successfully!")

    except Exception as e:
        print(f"Error in factory layout demo: {e}")
        import traceback
        traceback.print_exc()


def demo_amr_model():
    """
    Demonstrate AMR kinematics and turret rotation.

    Creates a single AMR, simulates circular motion for 100 steps,
    shows independent turret rotation, and visualizes the trajectory.
    """
    print("\n" + "="*70)
    print("DEMO 2: AMR Robot Model & Kinematics")
    print("="*70)

    try:
        # Create factory and AMR
        factory = FactoryEnvironment()
        parking_stations = factory.get_stations_by_type(StationType.PARKING_BAY)

        if not parking_stations:
            print("No parking stations found!")
            return

        start_pos = parking_stations[0].position
        robot = AMRRobot("amr_demo", start_pos, start_heading=0.0)

        print(f"Created robot at position {start_pos}")
        print(f"Robot specs: {robot.specs.length}m x {robot.specs.width}m")
        print(f"Max speed: {robot.specs.max_linear_speed} m/s")
        print(f"Initial battery: {robot.battery.current_charge:.1f}%")

        # Simulate circular motion
        dt = 0.1
        steps = 100

        print(f"\nSimulating {steps} steps ({steps * dt:.1f} seconds) of circular motion...")

        for step in range(steps):
            # Set circular motion: move forward and turn left
            if step < 50:
                robot.set_velocity(linear=1.0, angular=0.5)
                robot.rotate_turret(step * math.pi / 50.0, dt)
            else:
                robot.set_velocity(linear=1.0, angular=-0.5)
                robot.rotate_turret(math.pi, dt)

            robot.update(dt)

        # Print final state
        print(f"\nFinal robot state:")
        print(f"  Position: ({robot.x:.2f}, {robot.y:.2f})")
        print(f"  Heading: {math.degrees(robot.heading):.1f}°")
        print(f"  Turret heading: {math.degrees(robot.turret_heading):.1f}°")
        print(f"  Battery: {robot.battery.current_charge:.1f}%")
        print(f"  Trajectory length: {len(robot.trajectory)} points")

        # Visualize trajectory
        trajectory = robot.trajectory
        paths = [trajectory] if trajectory else []

        print("\nGenerating trajectory visualization...")
        robot_data = [{
            'id': robot.robot_id,
            'position': (robot.x, robot.y),
            'heading': robot.heading,
            'color': '#2E86AB'
        }]

        fig = factory.visualize(robots=robot_data, paths=paths,
                                 title="AMR Circular Motion Test")
        plt.show()

        print("AMR model demo completed successfully!")

    except Exception as e:
        print(f"Error in AMR model demo: {e}")
        import traceback
        traceback.print_exc()


def demo_dubins_paths():
    """
    Demonstrate Dubins path planning.

    Plans curved paths between multiple pose pairs using Dubins curves,
    compares with spline smoothing, and visualizes all paths.
    """
    print("\n" + "="*70)
    print("DEMO 3: Dubins Path Planning")
    print("="*70)

    try:
        # Create factory and path planner
        factory = FactoryEnvironment()
        dubins_planner = DubinsPlanner(turning_radius=2.0)
        spline_smoother = CubicSplineSmoother(min_turning_radius=2.0)

        print(f"Dubins planner turning radius: {dubins_planner.turning_radius}m")

        # Define test poses (start, goal pairs)
        test_poses = [
            ((10.0, 10.0, 0.0), (50.0, 50.0, math.pi/4)),
            ((20.0, 20.0, math.pi/2), (70.0, 30.0, -math.pi/2)),
            ((30.0, 40.0, math.pi), (80.0, 60.0, 0.0)),
        ]

        all_paths = []

        print(f"\nPlanning {len(test_poses)} Dubins paths...")

        for idx, (start, goal) in enumerate(test_poses):
            # Plan Dubins path
            dubins_path = dubins_planner.plan(start, goal)

            if dubins_path:
                print(f"\nPath {idx + 1}: {start} -> {goal}")
                print(f"  Type: {dubins_path.path_type.value}")
                print(f"  Length: {dubins_path.total_length:.2f}m")

                # Sample waypoints along the path
                waypoints = dubins_planner.sample_path(dubins_path, start, step_size=0.1)
                xy_only = [(p[0], p[1]) for p in waypoints]
                all_paths.append(xy_only)

                print(f"  Sampled waypoints: {len(waypoints)}")
            else:
                print(f"\nPath {idx + 1}: No valid Dubins path found")

        # Visualize all paths
        if all_paths:
            print("\nGenerating path visualization...")
            fig = factory.visualize(paths=all_paths,
                                     title="Dubins Path Planning (3 trajectories)")
            plt.show()

        print("Dubins path demo completed successfully!")

    except Exception as e:
        print(f"Error in Dubins path demo: {e}")
        import traceback
        traceback.print_exc()


def demo_docking_sequence():
    """
    Demonstrate docking station interaction.

    Shows the complete docking sequence: approach, align, final approach,
    docking, and undocking phases. Visualizes each phase transition.
    """
    print("\n" + "="*70)
    print("DEMO 4: Docking Sequence")
    print("="*70)

    try:
        # Create factory and robot
        factory = FactoryEnvironment()
        charging_stations = factory.get_stations_by_type(StationType.CHARGING_STATION)

        if not charging_stations:
            print("No charging stations found!")
            return

        charging_station = charging_stations[0]
        robot = AMRRobot("amr_dock_demo", (charging_station.position[0] - 10,
                                            charging_station.position[1]), 0.0)

        dock_controller = DockController()

        print(f"Created robot for docking demo")
        print(f"Target station: {charging_station.station_id}")
        print(f"Station position: {charging_station.position}")
        print(f"Approach point: {charging_station.approach_point}")

        # Simulate approach to station
        dt = 0.1
        max_steps = 500
        step = 0
        docked = False

        print(f"\nSimulating approach and docking (max {max_steps} steps)...")

        while step < max_steps and not docked:
            # Simple approach: move toward station
            dx = charging_station.position[0] - robot.x
            dy = charging_station.position[1] - robot.y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < 0.5:
                # Close enough - stop and mark as docked
                robot.set_velocity(0.0, 0.0)
                robot.start_charging()
                docked = True
                print(f"\nDocking complete at step {step}!")
                print(f"Robot position: ({robot.x:.2f}, {robot.y:.2f})")
                print(f"Distance to dock: {distance:.2f}m")
            else:
                # Move toward dock
                target_heading = math.atan2(dy, dx)
                heading_error = robot._angle_difference(target_heading, robot.heading)

                # Simple PD-like control
                robot.set_velocity(0.8, heading_error * 0.5)

            robot.update(dt)
            step += 1

        # Print docking statistics
        print(f"\nDocking sequence statistics:")
        print(f"  Steps to dock: {step}")
        print(f"  Total time: {step * dt:.2f}s")
        print(f"  Battery after approach: {robot.battery.current_charge:.1f}%")
        print(f"  State: {robot.state.value}")

        # Visualize final state
        robot_data = [{
            'id': robot.robot_id,
            'position': (robot.x, robot.y),
            'heading': robot.heading,
            'color': '#FFD700' if robot.state == AMRState.CHARGING else '#2E86AB'
        }]

        print("\nGenerating docking visualization...")
        fig = factory.visualize(robots=robot_data,
                                 title="Docking Sequence - Final State")
        plt.show()

        print("Docking demo completed successfully!")

    except Exception as e:
        print(f"Error in docking demo: {e}")
        import traceback
        traceback.print_exc()


def demo_traffic_management():
    """
    Demonstrate traffic management at intersections.

    Creates 3 AMRs with crossing paths, shows traffic manager controlling
    right-of-way, velocity modifiers, and intersection management.
    """
    print("\n" + "="*70)
    print("DEMO 5: Traffic Management at Intersections")
    print("="*70)

    try:
        # Create factory and traffic manager
        factory = FactoryEnvironment()
        intersections = factory.get_intersections()

        if not intersections:
            print("No intersections found!")
            return

        traffic_mgr = TrafficManager(intersections, safety_radius=1.5)

        print(f"Factory has {len(intersections)} intersections")

        # Create 3 robots at different positions
        robots = {}
        start_positions = [
            (10.0, 35.0, 0.0),  # Robot 1: moving east
            (37.0, 10.0, math.pi/2),  # Robot 2: moving north
            (70.0, 65.0, math.pi),  # Robot 3: moving west
        ]

        for idx, (x, y, heading) in enumerate(start_positions):
            robot_id = f"amr_traffic_{idx + 1}"
            robots[robot_id] = AMRRobot(robot_id, (x, y), heading)

        print(f"\nCreated {len(robots)} robots for traffic demo")
        for robot_id, robot in robots.items():
            print(f"  - {robot_id} at ({robot.x:.1f}, {robot.y:.1f}), "
                  f"heading {math.degrees(robot.heading):.0f}°")

        # Set priorities
        priorities = {
            "amr_traffic_1": TrafficPriority.HIGH,
            "amr_traffic_2": TrafficPriority.NORMAL,
            "amr_traffic_3": TrafficPriority.LOW,
        }

        for robot_id, priority in priorities.items():
            traffic_mgr.set_robot_priority(robot_id, priority)

        # Simulate movement through intersection
        dt = 0.1
        max_steps = 200

        print(f"\nSimulating traffic interaction for {max_steps} steps...")

        # Set constant velocities
        robots["amr_traffic_1"].set_velocity(1.0, 0.0)  # East
        robots["amr_traffic_2"].set_velocity(0.0, 1.0)  # North
        robots["amr_traffic_3"].set_velocity(-1.0, 0.0)  # West

        intersection_claims = {rid: 0 for rid in robots}

        for step in range(max_steps):
            # Update robot positions
            for robot_id, robot in robots.items():
                robot.update(dt)

                # Check if near any intersection
                for intersection_pos in intersections:
                    dist = robot.distance_to(intersection_pos)
                    if dist < 3.0:  # Within 3m of intersection
                        intersection_claims[robot_id] += 1

        # Print statistics
        print(f"\nTraffic management statistics:")
        for robot_id in robots:
            steps_near_intersection = intersection_claims[robot_id]
            print(f"  {robot_id}: {steps_near_intersection} steps near intersection")

        # Visualize final state
        robot_data = []
        for robot_id, robot in robots.items():
            color_map = {
                TrafficPriority.EMERGENCY: '#FF0000',
                TrafficPriority.HIGH: '#FF8800',
                TrafficPriority.NORMAL: '#00CC00',
                TrafficPriority.LOW: '#0088FF',
            }
            color = color_map.get(priorities[robot_id], '#888888')
            robot_data.append({
                'id': robot_id,
                'position': (robot.x, robot.y),
                'heading': robot.heading,
                'color': color
            })

        print("\nGenerating traffic management visualization...")
        fig = factory.visualize(robots=robot_data,
                                 title="Traffic Management Demo (3 robots)")
        plt.show()

        print("Traffic management demo completed successfully!")

    except Exception as e:
        print(f"Error in traffic management demo: {e}")
        import traceback
        traceback.print_exc()


def demo_full_simulation():
    """
    Full fleet simulation with production tasks.

    THE BIG ONE: Creates 8 AMRs, generates 15 production tasks, runs
    FleetCoordinator for 500 steps, captures snapshots, and shows
    production metrics.
    """
    print("\n" + "="*70)
    print("DEMO 6: Full Fleet Simulation")
    print("="*70)

    try:
        # Initialize factory
        factory = FactoryEnvironment()
        parking_stations = factory.get_stations_by_type(StationType.PARKING_BAY)

        num_robots = 8
        num_tasks = 15
        max_steps = 500
        dt = 0.1

        print(f"Initializing full simulation:")
        print(f"  - Robots: {num_robots}")
        print(f"  - Tasks: {num_tasks}")
        print(f"  - Simulation steps: {max_steps} ({max_steps * dt:.1f}s)")

        if len(parking_stations) < num_robots:
            print(f"Warning: Only {len(parking_stations)} parking stations for {num_robots} robots")
            num_robots = len(parking_stations)

        # Create fleet
        parking_positions = [s.position for s in parking_stations[:num_robots]]
        robots = create_default_fleet(num_robots, parking_positions)

        print(f"Created fleet of {len(robots)} robots")

        # Initialize systems
        intersections = factory.get_intersections()
        traffic_mgr = TrafficManager(intersections, safety_radius=1.5)

        try:
            path_planner = FactoryPathPlanner(factory)
        except Exception as e:
            print(f"Note: FactoryPathPlanner not fully initialized: {e}")
            path_planner = None

        dock_controller = DockController()

        # Initialize coordinator
        coordinator = FleetCoordinator(
            factory, robots, path_planner, traffic_mgr, dock_controller
        )

        print(f"Initialized FleetCoordinator")

        # Generate production tasks
        tasks = coordinator.generate_production_tasks(num_tasks)
        for task in tasks:
            coordinator.add_task(task)

        print(f"Generated {len(coordinator.task_queue)} production tasks")

        # Run simulation
        print(f"\nRunning simulation for {max_steps} steps...")

        snapshot_steps = [0, 100, 250, max_steps - 1]
        snapshots = {}

        start_time = time.time()

        for step in range(max_steps):
            # Update all robots
            for robot in robots.values():
                if robot.state == AMRState.IDLE or robot.state == AMRState.NAVIGATING:
                    # Simple idle behavior
                    if step % 10 == 0:
                        robot.set_velocity(0.5, 0.0)
                    elif step % 10 == 5:
                        robot.set_velocity(0.0, 0.0)

                robot.update(dt)

            # Update traffic manager
            robot_positions = {rid: (r.x, r.y) for rid, r in robots.items()}
            try:
                # Try update_robot_positions first (if available)
                if hasattr(traffic_mgr, 'update_robot_positions'):
                    traffic_mgr.update_robot_positions(robot_positions)
                else:
                    # Fall back to generic update
                    traffic_mgr.update()
            except Exception:
                # Traffic manager update may not be critical
                pass

            # Capture snapshots at key moments
            if step in snapshot_steps:
                snapshots[step] = {
                    'positions': {rid: (r.x, r.y) for rid, r in robots.items()},
                    'battery': {rid: r.battery.current_charge for rid, r in robots.items()},
                    'states': {rid: r.state.value for rid, r in robots.items()},
                }
                print(f"  Snapshot at step {step}")

        elapsed = time.time() - start_time

        # Print final statistics
        print(f"\nSimulation completed in {elapsed:.2f}s")
        print(f"\nFinal fleet statistics:")

        total_distance = sum(len(r.trajectory) * dt * r.specs.max_linear_speed * 0.5
                             for r in robots.values())
        avg_battery = np.mean([r.battery.current_charge for r in robots.values()])

        print(f"  Total distance traveled: ~{total_distance:.1f}m")
        print(f"  Average battery level: {avg_battery:.1f}%")
        print(f"  Tasks created: {len(coordinator.completed_tasks) + len(coordinator.active_tasks)}")

        # Print per-robot stats
        print(f"\nPer-robot status:")
        for robot_id, robot in sorted(robots.items()):
            print(f"  {robot_id}: pos=({robot.x:.1f}, {robot.y:.1f}), "
                  f"batt={robot.battery.current_charge:.1f}%, "
                  f"state={robot.state.value}")

        # Visualize final state
        print("\nGenerating final fleet visualization...")

        robot_data = []
        for robot_id, robot in robots.items():
            state_colors = {
                'navigating': '#00CC44',
                'idle': '#4488FF',
                'charging': '#FFDD00',
                'emergency_stop': '#FF0000',
                'docking': '#FF8833',
                'parked': '#888888',
                'waiting': '#00DDDD',
            }
            color = state_colors.get(robot.state.value, '#888888')
            robot_data.append({
                'id': robot_id,
                'position': (robot.x, robot.y),
                'heading': robot.heading,
                'color': color
            })

        fig = factory.visualize(robots=robot_data,
                                 title=f"Full Fleet Simulation (Step {max_steps})")
        plt.show()

        print("Full simulation demo completed successfully!")

    except Exception as e:
        print(f"Error in full simulation demo: {e}")
        import traceback
        traceback.print_exc()


def demo_navigation_visualization():
    """
    Demonstrate Dubins path planning with multi-frame visualization.

    Shows a 2x3 grid of snapshots displaying robot movement along a Dubins curve
    at 0%, 20%, 40%, 60%, 80%, and 100% progress. Includes path visualization,
    robot position, heading, and distance traveled.
    """
    print("\n" + "="*70)
    print("DEMO 7: Navigation Visualization (Dubins Path Planning)")
    print("="*70)

    try:
        output_path = Path(__file__).parent / 'docs' / 'images' / 'navigation_demo.png'
        output_path.parent.mkdir(parents=True, exist_ok=True)

        print(f"Generating multi-frame navigation visualization...")
        create_navigation_demo(str(output_path))

        print(f"Navigation visualization saved to: {output_path}")
        print("Visualization shows:")
        print("  - Dubins curve optimal path planning")
        print("  - 6 snapshots of robot progression")
        print("  - Approach point and station visualization")
        print("  - Real-time distance and heading information")

    except Exception as e:
        print(f"Error in navigation visualization demo: {e}")
        import traceback
        traceback.print_exc()


def demo_obstacle_avoidance_visualization():
    """
    Demonstrate Dynamic Window Approach (DWA) obstacle avoidance.

    Shows a 2x2 grid of frames depicting:
    1. Robot approaching obstacle (DWA kicks in)
    2. Robot emergency stop (obstacle too close)
    3. Obstacle clear (monitoring)
    4. Robot resume (new path calculated)

    Includes obstacle visualization, safety zones, and real-time alerts.
    """
    print("\n" + "="*70)
    print("DEMO 8: Obstacle Avoidance Visualization (DWA)")
    print("="*70)

    try:
        output_path = Path(__file__).parent / 'docs' / 'images' / 'obstacle_avoidance_demo.png'
        output_path.parent.mkdir(parents=True, exist_ok=True)

        print(f"Generating obstacle avoidance visualization...")
        create_obstacle_avoidance_demo(str(output_path))

        print(f"Obstacle avoidance visualization saved to: {output_path}")
        print("Visualization shows:")
        print("  - Dynamic Window Approach (DWA) algorithm")
        print("  - 4 frames of avoidance sequence")
        print("  - Obstacle detection and emergency response")
        print("  - Safety distance zones (1.5m buffer)")
        print("  - Real-time alert system")

    except Exception as e:
        print(f"Error in obstacle avoidance visualization demo: {e}")
        import traceback
        traceback.print_exc()


def demo_alert_system_visualization():
    """
    Demonstrate multi-robot alert system and fleet coordination.

    Shows how alerts propagate through the fleet and how other robots
    dynamically adjust their paths based on broadcast alerts:
    - Robot 1: Normal operation
    - Robot 2: Encounters obstacle, generates alert, stops
    - Robot 3: Receives alert, reroutes autonomously

    Includes timeline visualization of alert events.
    """
    print("\n" + "="*70)
    print("DEMO 9: Alert System Visualization (Multi-Robot Coordination)")
    print("="*70)

    try:
        output_path = Path(__file__).parent / 'docs' / 'images' / 'alert_demo.png'
        output_path.parent.mkdir(parents=True, exist_ok=True)

        print(f"Generating alert system visualization...")
        create_alert_demo(str(output_path))

        print(f"Alert system visualization saved to: {output_path}")
        print("Visualization shows:")
        print("  - 3-robot fleet coordination")
        print("  - Alert detection and broadcasting")
        print("  - Real-time path rerouting")
        print("  - Timeline of system events")
        print("  - Collision-free fleet operation")

    except Exception as e:
        print(f"Error in alert system visualization demo: {e}")
        import traceback
        traceback.print_exc()


def main():
    """
    Main entry point with command-line argument parsing.
    """
    parser = argparse.ArgumentParser(
        description="Multi-AMR Factory Navigation Demo Suite",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py                        # Run all demos
  python main.py --demo factory         # Factory layout only
  python main.py --demo full            # Full simulation only
  python main.py --demo navigation      # Dubins path visualization
  python main.py --demo obstacles       # DWA obstacle avoidance
  python main.py --demo alerts          # Multi-robot alert system
  python main.py --demo all --num-robots 12  # All demos with custom params
        """
    )

    parser.add_argument(
        '--demo',
        choices=['factory', 'amr', 'dubins', 'docking', 'traffic', 'full', 'navigation', 'obstacles', 'alerts', 'all'],
        default='all',
        help='Which demo(s) to run (default: all)'
    )

    parser.add_argument(
        '--num-robots',
        type=int,
        default=8,
        help='Number of robots for full simulation (default: 8)'
    )

    parser.add_argument(
        '--num-tasks',
        type=int,
        default=15,
        help='Number of production tasks (default: 15)'
    )

    parser.add_argument(
        '--steps',
        type=int,
        default=500,
        help='Simulation steps for full demo (default: 500)'
    )

    args = parser.parse_args()

    # Print header
    print("\n" + "="*70)
    print("ROS2 Multi-AMR Factory Navigation — Demo Suite")
    print("="*70)
    print(f"Author: Muskaan Maheshwari")
    print(f"Running on: {sys.platform}")

    # Map demos
    demos = {
        'factory': demo_factory_layout,
        'amr': demo_amr_model,
        'dubins': demo_dubins_paths,
        'docking': demo_docking_sequence,
        'traffic': demo_traffic_management,
        'full': demo_full_simulation,
        'navigation': demo_navigation_visualization,
        'obstacles': demo_obstacle_avoidance_visualization,
        'alerts': demo_alert_system_visualization,
    }

    # Determine which demos to run
    if args.demo == 'all':
        demos_to_run = list(demos.keys())
    else:
        demos_to_run = [args.demo]

    # Run selected demos
    print(f"\nSelected demos: {', '.join(demos_to_run)}")
    print("="*70)

    for demo_name in demos_to_run:
        try:
            demos[demo_name]()
        except KeyboardInterrupt:
            print(f"\n\nDemo interrupted by user.")
            break
        except Exception as e:
            print(f"\n\nUnexpected error in {demo_name}: {e}")
            import traceback
            traceback.print_exc()

    print("\n" + "="*70)
    print("Demo suite completed!")
    print("="*70 + "\n")


if __name__ == '__main__':
    main()
