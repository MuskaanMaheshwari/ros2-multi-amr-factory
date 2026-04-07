"""
Fleet Coordinator — Central orchestration system for multi-AMR factory operations.

Manages task assignment, scheduling, battery management, and collision avoidance
across the entire fleet of autonomous robots.

Author: Muskaan Maheshwari
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple
from collections import defaultdict
import heapq
import time
import math


class TaskType(Enum):
    """Types of tasks robots can perform."""
    MATERIAL_TRANSPORT = auto()
    CHARGING = auto()
    PARKING = auto()
    REPOSITIONING = auto()


class TaskPriority(Enum):
    """Priority levels for task execution."""
    CRITICAL = 0
    HIGH = 1
    NORMAL = 2
    LOW = 3


@dataclass
class Task:
    """Represents a unit of work in the factory."""
    task_id: str
    task_type: TaskType
    priority: TaskPriority = TaskPriority.NORMAL
    pickup_station_id: Optional[str] = None
    dropoff_station_id: Optional[str] = None
    payload_kg: float = 100.0
    assigned_robot_id: Optional[str] = None
    status: str = 'pending'  # pending, assigned, in_progress, pickup, transport, dropoff, completed, failed
    created_time: float = 0.0
    assigned_time: float = 0.0
    completed_time: float = 0.0
    estimated_distance: float = 0.0

    def __lt__(self, other: 'Task') -> bool:
        """Enable priority queue ordering: priority first, then creation time."""
        if self.priority.value != other.priority.value:
            return self.priority.value < other.priority.value
        return self.created_time < other.created_time


class FleetCoordinator:
    """
    Central brain of the multi-AMR factory system.
    
    Orchestrates task assignment, scheduling, battery management, and
    collision avoidance for all robots in the fleet.
    """

    # Production flow: from one station to the next
    PRODUCTION_FLOW = [
        ('INCOMING_MATERIAL', 'CELL_ASSEMBLY'),
        ('CELL_ASSEMBLY', 'MODULE_PACKING'),
        ('MODULE_PACKING', 'PACK_INTEGRATION'),
        ('PACK_INTEGRATION', 'TESTING_QC'),
        ('TESTING_QC', 'SHIPPING'),
    ]

    def __init__(self, factory_env, robots: Dict, path_planner, traffic_manager, dock_controller):
        """
        Initialize the fleet coordinator.
        
        Args:
            factory_env: Factory environment with station locations
            robots: Dict of robot_id -> AMRRobot instances
            path_planner: Path planning module
            traffic_manager: Traffic management system
            dock_controller: Docking sequence controller
        """
        self.factory_env = factory_env
        self.robots = robots
        self.path_planner = path_planner
        self.traffic_manager = traffic_manager
        self.dock_controller = dock_controller

        # Task management
        self.task_queue: List[Task] = []  # Priority queue as list
        self.active_tasks: Dict[str, Task] = {}  # task_id -> Task
        self.completed_tasks: List[Task] = []
        self.failed_tasks: List[Task] = []

        # Robot state tracking
        self.robot_tasks: Dict[str, str] = {}  # robot_id -> task_id
        self.robot_paths: Dict[str, List] = defaultdict(list)  # robot_id -> list of waypoints
        self.robot_path_index: Dict[str, int] = defaultdict(int)

        # Docking sequences
        self.active_docking_sequences: Dict[str, Dict] = {}  # robot_id -> docking_state

        # Statistics
        self.task_counter = 0
        self.start_time = 0.0
        self.total_distance_traveled: float = 0.0
        self.deadlock_count = 0
        self.emergency_stop_count = 0

        # Configuration
        self.battery_charge_threshold = 30.0  # Charge when battery < 30%
        self.battery_target_charge = 80.0
        self.task_assignment_interval = 0.5  # seconds
        self.deadlock_check_interval = 2.0  # seconds
        self.last_assignment_time = 0.0
        self.last_deadlock_check = 0.0

    def add_task(self, task: Task) -> None:
        """
        Add a task to the priority queue.
        
        Args:
            task: Task instance to add
        """
        task.created_time = time.time() if self.start_time == 0 else self.start_time
        heapq.heappush(self.task_queue, task)

    def generate_production_tasks(self, num_tasks: int = 10) -> List[Task]:
        """
        Auto-generate realistic production tasks following the factory flow.
        
        Battery factory production flow:
        INCOMING_MATERIAL -> CELL_ASSEMBLY -> MODULE_PACKING -> PACK_INTEGRATION -> TESTING_QC -> SHIPPING
        
        Args:
            num_tasks: Number of tasks to generate
            
        Returns:
            List of generated Task instances
        """
        generated_tasks = []
        current_time = time.time() if self.start_time == 0 else self.start_time

        for i in range(num_tasks):
            # Randomly select a flow step
            flow_idx = i % len(self.PRODUCTION_FLOW)
            pickup_station, dropoff_station = self.PRODUCTION_FLOW[flow_idx]

            # Determine priority (earlier tasks get higher priority)
            if i < num_tasks * 0.2:  # First 20%
                priority = TaskPriority.CRITICAL
            elif i < num_tasks * 0.5:  # Next 30%
                priority = TaskPriority.HIGH
            elif i < num_tasks * 0.8:  # Next 30%
                priority = TaskPriority.NORMAL
            else:
                priority = TaskPriority.LOW

            task = Task(
                task_id=f"TASK_{self.task_counter:04d}",
                task_type=TaskType.MATERIAL_TRANSPORT,
                priority=priority,
                pickup_station_id=pickup_station,
                dropoff_station_id=dropoff_station,
                payload_kg=100.0,
                created_time=current_time + i * 0.1,  # Stagger creation
            )
            self.task_counter += 1
            generated_tasks.append(task)
            self.add_task(task)

        return generated_tasks

    def assign_tasks(self) -> None:
        """
        Assign pending tasks to available robots.
        
        For each pending task, finds the best available robot based on:
        - Robot must be IDLE or PARKED
        - Robot must have sufficient battery
        - Score = 1 / (distance_to_pickup + battery_penalty)
        
        Also auto-assigns CHARGING tasks for low-battery robots.
        """
        current_time = time.time() if self.start_time == 0 else self.start_time

        # Collect pending tasks (those in priority queue)
        pending_tasks = list(self.task_queue)

        for task in pending_tasks:
            if task.status != 'pending':
                continue

            # Skip if already assigned
            if task.assigned_robot_id is not None:
                continue

            # Find best robot
            best_robot_id = None
            best_score = -float('inf')

            for robot_id, robot in self.robots.items():
                # Check robot state
                if robot.state not in ['IDLE', 'PARKED']:
                    continue

                # Check if robot already has a task
                if robot_id in self.robot_tasks:
                    continue

                # Get pickup station location
                if not hasattr(self.factory_env, 'stations'):
                    continue

                pickup_pos = self.factory_env.stations.get(task.pickup_station_id)
                if pickup_pos is None:
                    continue

                # Calculate distance from robot to pickup
                robot_pos = (robot.x, robot.y)
                dist_to_pickup = self._euclidean_distance(robot_pos, pickup_pos)

                # Check battery: must reach pickup + dropoff + charger
                if task.dropoff_station_id:
                    dropoff_pos = self.factory_env.stations.get(task.dropoff_station_id)
                    if dropoff_pos:
                        dist_to_dropoff = self._euclidean_distance(pickup_pos, dropoff_pos)
                    else:
                        dist_to_dropoff = 0.0
                else:
                    dist_to_dropoff = 0.0

                required_distance = dist_to_pickup + dist_to_dropoff
                battery_needed = required_distance * 0.05  # Rough estimate: 0.05% per meter

                if robot.battery < battery_needed + 20.0:  # Leave 20% buffer
                    continue

                # Calculate score
                battery_penalty = 0.0
                if robot.battery < 50.0:
                    battery_penalty = (50.0 - robot.battery) * 0.5

                if dist_to_pickup > 0:
                    score = 1.0 / (dist_to_pickup + 1.0 + battery_penalty)
                else:
                    score = 100.0 - battery_penalty

                if score > best_score:
                    best_score = score
                    best_robot_id = robot_id

            # Assign task if suitable robot found
            if best_robot_id:
                task.assigned_robot_id = best_robot_id
                task.status = 'assigned'
                task.assigned_time = current_time
                self.robot_tasks[best_robot_id] = task.task_id
                self.active_tasks[task.task_id] = task
                heapq.heappop(self.task_queue)

        # Auto-assign charging tasks for low-battery robots
        for robot_id, robot in self.robots.items():
            if robot_id in self.robot_tasks:
                continue  # Already has a task

            if robot.battery < self.battery_charge_threshold:
                # Get charger location (assume CHARGER_1 exists)
                charger_pos = self.factory_env.stations.get('CHARGER_1')
                if charger_pos is None:
                    charger_pos = self.factory_env.stations.get('PARKING_BAY')

                if charger_pos:
                    charge_task = Task(
                        task_id=f"CHARGE_{robot_id}_{int(current_time)}",
                        task_type=TaskType.CHARGING,
                        priority=TaskPriority.HIGH,
                        dropoff_station_id='CHARGER_1',
                        created_time=current_time,
                    )
                    charge_task.assigned_robot_id = robot_id
                    charge_task.status = 'assigned'
                    charge_task.assigned_time = current_time
                    self.robot_tasks[robot_id] = charge_task.task_id
                    self.active_tasks[charge_task.task_id] = charge_task

    def update(self, dt: float) -> Dict:
        """
        Main simulation loop step.
        
        Updates all robots, manages task transitions, handles collisions,
        and tracks fleet statistics.
        
        Args:
            dt: Time delta in seconds
            
        Returns:
            Dict with fleet statistics and status
        """
        current_time = time.time() if self.start_time == 0 else self.start_time

        # Initialize start time
        if self.start_time == 0:
            self.start_time = current_time

        # Periodic task assignment
        if current_time - self.last_assignment_time >= self.task_assignment_interval:
            self.assign_tasks()
            self.last_assignment_time = current_time

        # Update each robot
        for robot_id, robot in self.robots.items():
            task_id = self.robot_tasks.get(robot_id)
            task = self.active_tasks.get(task_id) if task_id else None

            if task is None:
                # Idle robot
                robot.state = 'IDLE'
                robot.velocity = 0.0
                continue

            # Execute task state machine
            if task.task_type == TaskType.CHARGING:
                self._handle_charging(robot_id, robot, task, dt)
            elif task.task_type == TaskType.MATERIAL_TRANSPORT:
                self._handle_transport(robot_id, robot, task, dt)
            elif task.task_type == TaskType.PARKING:
                self._handle_parking(robot_id, robot, task, dt)
            elif task.task_type == TaskType.REPOSITIONING:
                self._handle_repositioning(robot_id, robot, task, dt)

        # Update docking sequences
        self._update_docking_sequences(dt)

        # Update traffic manager
        self.traffic_manager.update(dt)

        # Check for deadlocks
        if current_time - self.last_deadlock_check >= self.deadlock_check_interval:
            self._check_and_resolve_deadlocks()
            self.last_deadlock_check = current_time

        # Compile statistics
        stats = self.get_fleet_status()
        stats.update(self.get_production_metrics())

        return stats

    def _handle_transport(self, robot_id: str, robot, task: Task, dt: float) -> None:
        """Handle material transport task state machine."""
        # Start navigation if needed
        if task.status == 'assigned':
            if not self.robot_paths[robot_id]:
                # Plan path to pickup station
                pickup_pos = self.factory_env.stations.get(task.pickup_station_id)
                if pickup_pos:
                    path = self.path_planner.plan_path(
                        (robot.x, robot.y),
                        pickup_pos,
                        obstacles=self.factory_env.obstacles if hasattr(self.factory_env, 'obstacles') else []
                    )
                    if path:
                        self.robot_paths[robot_id] = path
                        self.robot_path_index[robot_id] = 0
                        task.status = 'in_progress'
                        robot.state = 'NAVIGATING'

        # Navigate to pickup
        if task.status == 'in_progress' and task.pickup_station_id:
            self._navigate_robot(robot_id, robot, dt)

            # Check if reached pickup
            pickup_pos = self.factory_env.stations.get(task.pickup_station_id)
            if pickup_pos and self._at_location(robot, pickup_pos, tolerance=0.5):
                task.status = 'pickup'
                robot.state = 'DOCKING'
                self.robot_paths[robot_id] = []
                self.robot_path_index[robot_id] = 0

        # Dock and load at pickup
        if task.status == 'pickup':
            docking_complete = self._perform_docking(robot_id, robot, task.pickup_station_id, dt)
            if docking_complete:
                # Load payload
                robot.payload = task.payload_kg
                task.status = 'transport'
                robot.state = 'NAVIGATING'

        # Navigate to dropoff
        if task.status == 'transport' and task.dropoff_station_id:
            if not self.robot_paths[robot_id]:
                dropoff_pos = self.factory_env.stations.get(task.dropoff_station_id)
                if dropoff_pos:
                    path = self.path_planner.plan_path(
                        (robot.x, robot.y),
                        dropoff_pos,
                        obstacles=self.factory_env.obstacles if hasattr(self.factory_env, 'obstacles') else []
                    )
                    if path:
                        self.robot_paths[robot_id] = path
                        self.robot_path_index[robot_id] = 0

            self._navigate_robot(robot_id, robot, dt)

            # Check if reached dropoff
            dropoff_pos = self.factory_env.stations.get(task.dropoff_station_id)
            if dropoff_pos and self._at_location(robot, dropoff_pos, tolerance=0.5):
                task.status = 'dropoff'
                robot.state = 'DOCKING'
                self.robot_paths[robot_id] = []
                self.robot_path_index[robot_id] = 0

        # Dock and unload at dropoff
        if task.status == 'dropoff':
            docking_complete = self._perform_docking(robot_id, robot, task.dropoff_station_id, dt)
            if docking_complete:
                # Unload payload
                robot.payload = 0.0
                task.status = 'completed'
                task.completed_time = time.time() if self.start_time == 0 else self.start_time
                self.completed_tasks.append(task)
                self.active_tasks.pop(task.task_id, None)
                self.robot_tasks.pop(robot_id, None)
                robot.state = 'IDLE'

    def _handle_charging(self, robot_id: str, robot, task: Task, dt: float) -> None:
        """Handle charging task."""
        charger_pos = self.factory_env.stations.get('CHARGER_1')
        if charger_pos is None:
            charger_pos = self.factory_env.stations.get('PARKING_BAY')

        # Navigate to charger
        if task.status == 'assigned':
            if not self.robot_paths[robot_id]:
                if charger_pos:
                    path = self.path_planner.plan_path(
                        (robot.x, robot.y),
                        charger_pos,
                        obstacles=self.factory_env.obstacles if hasattr(self.factory_env, 'obstacles') else []
                    )
                    if path:
                        self.robot_paths[robot_id] = path
                        self.robot_path_index[robot_id] = 0
                        task.status = 'in_progress'
                        robot.state = 'NAVIGATING'

        if task.status == 'in_progress':
            self._navigate_robot(robot_id, robot, dt)

            if charger_pos and self._at_location(robot, charger_pos, tolerance=0.5):
                task.status = 'pickup'
                robot.state = 'CHARGING'
                self.robot_paths[robot_id] = []

        # Charge battery
        if task.status == 'pickup':
            robot.battery = min(100.0, robot.battery + 10.0 * dt)  # 10% per second

            if robot.battery >= self.battery_target_charge:
                task.status = 'completed'
                task.completed_time = time.time() if self.start_time == 0 else self.start_time
                self.completed_tasks.append(task)
                self.active_tasks.pop(task.task_id, None)
                self.robot_tasks.pop(robot_id, None)
                robot.state = 'IDLE'

    def _handle_parking(self, robot_id: str, robot, task: Task, dt: float) -> None:
        """Handle parking task."""
        parking_pos = self.factory_env.stations.get('PARKING_BAY')
        if parking_pos is None:
            task.status = 'completed'
            return

        if task.status == 'assigned':
            if not self.robot_paths[robot_id]:
                path = self.path_planner.plan_path(
                    (robot.x, robot.y),
                    parking_pos,
                    obstacles=self.factory_env.obstacles if hasattr(self.factory_env, 'obstacles') else []
                )
                if path:
                    self.robot_paths[robot_id] = path
                    self.robot_path_index[robot_id] = 0
                    task.status = 'in_progress'
                    robot.state = 'NAVIGATING'

        if task.status == 'in_progress':
            self._navigate_robot(robot_id, robot, dt)

            if self._at_location(robot, parking_pos, tolerance=0.5):
                robot.state = 'PARKED'
                task.status = 'completed'
                task.completed_time = time.time() if self.start_time == 0 else self.start_time
                self.completed_tasks.append(task)
                self.active_tasks.pop(task.task_id, None)
                self.robot_tasks.pop(robot_id, None)

    def _handle_repositioning(self, robot_id: str, robot, task: Task, dt: float) -> None:
        """Handle repositioning task."""
        # Similar to parking but with a calculated staging position
        # For now, treat as parking
        self._handle_parking(robot_id, robot, task, dt)

    def _navigate_robot(self, robot_id: str, robot, dt: float) -> None:
        """
        Move robot along its planned path.
        
        Args:
            robot_id: Robot identifier
            robot: Robot instance
            dt: Time delta in seconds
        """
        path = self.robot_paths.get(robot_id, [])
        if not path:
            return

        path_idx = self.robot_path_index.get(robot_id, 0)
        if path_idx >= len(path):
            return

        # Get current and target waypoints
        current_pos = (robot.x, robot.y)
        target_pos = path[path_idx]

        # Calculate direction
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.1:  # Reached waypoint
            self.robot_path_index[robot_id] = path_idx + 1
            if self.robot_path_index[robot_id] >= len(path):
                robot.velocity = 0.0
            return

        # Calculate desired heading
        desired_heading = math.atan2(dy, dx)

        # Smooth heading change
        current_heading = robot.heading
        heading_diff = self._normalize_angle(desired_heading - current_heading)
        max_turn_rate = 1.0  # rad/s
        heading_change = max(-max_turn_rate * dt, min(heading_diff, max_turn_rate * dt))
        robot.heading = self._normalize_angle(current_heading + heading_change)

        # Set velocity with traffic modifiers
        base_velocity = 0.5  # m/s
        traffic_modifier = self.traffic_manager.get_velocity_modifier(robot_id, (robot.x, robot.y))
        robot.velocity = base_velocity * traffic_modifier

        # Slow down near waypoint transitions
        if distance < 1.0:
            robot.velocity *= distance

        # Update robot position
        if robot.velocity > 0:
            robot.x += robot.velocity * dt * math.cos(robot.heading)
            robot.y += robot.velocity * dt * math.sin(robot.heading)
            self.total_distance_traveled += robot.velocity * dt

        # Update battery (consumption)
        robot.battery = max(0.0, robot.battery - 0.02 * robot.velocity * dt)

    def _perform_docking(self, robot_id: str, robot, station_id: str, dt: float) -> bool:
        """
        Perform docking sequence at a station.
        
        Args:
            robot_id: Robot identifier
            robot: Robot instance
            station_id: Station identifier
            dt: Time delta in seconds
            
        Returns:
            True if docking complete, False otherwise
        """
        if robot_id not in self.active_docking_sequences:
            # Start docking sequence
            self.active_docking_sequences[robot_id] = {
                'station_id': station_id,
                'state': 'aligning',
                'time_elapsed': 0.0,
            }

        sequence = self.active_docking_sequences[robot_id]
        sequence['time_elapsed'] += dt

        # Simulate docking process (5 seconds total)
        if sequence['time_elapsed'] < 5.0:
            return False
        else:
            # Docking complete
            self.active_docking_sequences.pop(robot_id, None)
            return True

    def _update_docking_sequences(self, dt: float) -> None:
        """Update active docking sequences."""
        # Already handled in _perform_docking; this is for cleanup
        pass

    def _check_and_resolve_deadlocks(self) -> None:
        """
        Check for robot deadlocks and attempt resolution.
        
        Simple heuristic: if two robots are close and both navigating,
        assign one to wait.
        """
        robot_ids = list(self.robots.keys())

        for i, robot_id_1 in enumerate(robot_ids):
            for robot_id_2 in robot_ids[i+1:]:
                robot_1 = self.robots[robot_id_1]
                robot_2 = self.robots[robot_id_2]

                if robot_1.state != 'NAVIGATING' or robot_2.state != 'NAVIGATING':
                    continue

                # Check if close
                dist = self._euclidean_distance((robot_1.x, robot_1.y), (robot_2.x, robot_2.y))
                if dist < 1.0:  # Within 1 meter
                    # Simple resolution: stop lower-priority robot
                    task_1 = self.active_tasks.get(self.robot_tasks.get(robot_id_1))
                    task_2 = self.active_tasks.get(self.robot_tasks.get(robot_id_2))

                    if task_1 and task_2:
                        if task_1.priority.value >= task_2.priority.value:
                            robot_2.velocity = 0.0
                        else:
                            robot_1.velocity = 0.0
                        self.deadlock_count += 1

    def get_fleet_status(self) -> Dict:
        """
        Get comprehensive fleet status.
        
        Returns:
            Dict with fleet composition and robot states
        """
        idle_count = 0
        navigating_count = 0
        docking_count = 0
        charging_count = 0
        parked_count = 0
        waiting_count = 0

        robot_details = []

        for robot_id, robot in self.robots.items():
            if robot.state == 'IDLE':
                idle_count += 1
            elif robot.state == 'NAVIGATING':
                navigating_count += 1
            elif robot.state == 'DOCKING':
                docking_count += 1
            elif robot.state == 'CHARGING':
                charging_count += 1
            elif robot.state == 'PARKED':
                parked_count += 1
            elif robot.state == 'WAITING':
                waiting_count += 1

            task_id = self.robot_tasks.get(robot_id, 'None')
            task = self.active_tasks.get(task_id) if task_id != 'None' else None

            robot_details.append({
                'robot_id': robot_id,
                'position': (robot.x, robot.y),
                'heading': robot.heading,
                'battery': robot.battery,
                'state': robot.state,
                'velocity': robot.velocity,
                'payload': robot.payload,
                'current_task': task_id,
                'task_status': task.status if task else 'none',
            })

        return {
            'total_robots': len(self.robots),
            'idle': idle_count,
            'navigating': navigating_count,
            'docking': docking_count,
            'charging': charging_count,
            'parked': parked_count,
            'waiting': waiting_count,
            'robot_details': robot_details,
            'tasks': {
                'pending': len(self.task_queue),
                'active': len(self.active_tasks),
                'completed': len(self.completed_tasks),
                'failed': len(self.failed_tasks),
            },
        }

    def get_production_metrics(self) -> Dict:
        """
        Get production performance metrics.
        
        Returns:
            Dict with throughput, utilization, and other KPIs
        """
        elapsed_time = time.time() - self.start_time if self.start_time > 0 else 0.001

        if elapsed_time > 0:
            tasks_per_hour = (len(self.completed_tasks) / elapsed_time) * 3600.0
        else:
            tasks_per_hour = 0.0

        if len(self.completed_tasks) > 0:
            total_task_time = sum(
                t.completed_time - t.assigned_time for t in self.completed_tasks
                if t.assigned_time > 0 and t.completed_time > 0
            )
            avg_task_time = total_task_time / len(self.completed_tasks)
        else:
            avg_task_time = 0.0

        if len(self.robots) > 0:
            active_robots = sum(1 for r in self.robots.values() if r.state != 'IDLE' and r.state != 'PARKED')
            fleet_utilization = (active_robots / len(self.robots)) * 100.0
        else:
            fleet_utilization = 0.0

        return {
            'throughput_tasks_per_hour': tasks_per_hour,
            'average_task_time': avg_task_time,
            'fleet_utilization_percent': fleet_utilization,
            'total_distance_traveled': self.total_distance_traveled,
            'deadlocks_resolved': self.deadlock_count,
            'emergency_stops': self.emergency_stop_count,
        }

    @staticmethod
    def _euclidean_distance(pos1: Tuple, pos2: Tuple) -> float:
        """Calculate Euclidean distance between two positions."""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    @staticmethod
    def _at_location(robot, location: Tuple, tolerance: float = 0.5) -> bool:
        """Check if robot is at a location within tolerance."""
        dist = FleetCoordinator._euclidean_distance((robot.x, robot.y), location)
        return dist <= tolerance

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
