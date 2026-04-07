"""
Docking station interaction system.

Handles precise approach, alignment, and docking sequences for factory AMRs
interfacing with charging stations, loading docks, unloading docks, and parking bays.

Author: Muskaan Maheshwari
"""

from typing import Tuple, Dict, List, Optional
from enum import Enum
from dataclasses import dataclass, field
import math
import time


class DockingPhase(Enum):
    """Phases of the docking sequence state machine."""
    APPROACH = "approach"
    ALIGNMENT = "alignment"
    TURRET_ALIGN = "turret_align"
    FINAL_APPROACH = "final_approach"
    DOCKED = "docked"
    UNDOCKING = "undocking"
    COMPLETE = "complete"


class StationType(Enum):
    """Types of docking stations in the factory."""
    CHARGING_STATION = "charging_station"
    LOADING_DOCK = "loading_dock"
    UNLOADING_DOCK = "unloading_dock"
    PARKING_BAY = "parking_bay"


@dataclass
class DockingSequence:
    """State and parameters for an active docking sequence."""
    station_id: str
    station_type: StationType
    phases: List[DockingPhase] = field(default_factory=list)
    current_phase: DockingPhase = DockingPhase.APPROACH
    phase_start_time: float = 0.0
    approach_point: Tuple[float, float] = (0.0, 0.0)
    dock_point: Tuple[float, float] = (0.0, 0.0)
    dock_heading: float = 0.0
    turret_target: float = 0.0
    docking_speed: float = 0.1
    alignment_tolerance: float = 0.02
    position_tolerance: float = 0.01
    operation_duration: float = 0.0
    operation_start_time: float = 0.0
    charge_start_battery: float = 0.0


class DockController:
    """
    Controls precise docking sequences for AMRs at factory stations.

    Manages state machine transitions, velocity commands, and alignment
    for different station types with submillimeter precision.
    """

    # Control parameters
    BASE_ALIGNMENT_KP = 2.0
    TURRET_ALIGNMENT_KP = 1.5
    MAX_ANGULAR_VELOCITY = 1.0  # rad/s
    MAX_TURRET_VELOCITY = 1.0  # rad/s

    # Thresholds
    APPROACH_DISTANCE_THRESHOLD = 0.3  # meters
    UNDOCKING_DISTANCE = 1.0  # meters

    def __init__(self, position_tolerance: float = 0.01, alignment_tolerance: float = 0.02):
        """
        Initialize the docking controller.

        Args:
            position_tolerance: Positional accuracy in meters (default ±10mm)
            alignment_tolerance: Heading alignment tolerance in radians (~1° = 0.02 rad)
        """
        self.position_tolerance = position_tolerance
        self.alignment_tolerance = alignment_tolerance

    def start_docking(self, robot, station) -> DockingSequence:
        """
        Initiate a docking sequence for a robot at a station.

        Creates a DockingSequence with appropriate phases and parameters
        based on the station type.

        Args:
            robot: AMRRobot instance with position, heading, battery attributes
            station: Station instance with position, orientation, approach attributes

        Returns:
            DockingSequence: Active docking sequence ready for updates
        """
        # Determine phase sequence based on station type
        if station.station_type == StationType.CHARGING_STATION:
            phases = [
                DockingPhase.APPROACH,
                DockingPhase.ALIGNMENT,
                DockingPhase.FINAL_APPROACH,
                DockingPhase.DOCKED,
                DockingPhase.UNDOCKING,
                DockingPhase.COMPLETE,
            ]
            operation_duration = 300.0  # Dynamic (15 min nominal)

        elif station.station_type == StationType.LOADING_DOCK:
            phases = [
                DockingPhase.APPROACH,
                DockingPhase.ALIGNMENT,
                DockingPhase.TURRET_ALIGN,
                DockingPhase.FINAL_APPROACH,
                DockingPhase.DOCKED,
                DockingPhase.UNDOCKING,
                DockingPhase.COMPLETE,
            ]
            operation_duration = 15.0  # 15 seconds for loading

        elif station.station_type == StationType.UNLOADING_DOCK:
            phases = [
                DockingPhase.APPROACH,
                DockingPhase.ALIGNMENT,
                DockingPhase.TURRET_ALIGN,
                DockingPhase.FINAL_APPROACH,
                DockingPhase.DOCKED,
                DockingPhase.UNDOCKING,
                DockingPhase.COMPLETE,
            ]
            operation_duration = 10.0  # 10 seconds for unloading

        else:  # PARKING_BAY
            phases = [
                DockingPhase.APPROACH,
                DockingPhase.ALIGNMENT,
                DockingPhase.FINAL_APPROACH,
                DockingPhase.DOCKED,
                DockingPhase.COMPLETE,
            ]
            operation_duration = 0.0  # No operation

        # Create sequence
        sequence = DockingSequence(
            station_id=station.station_id,
            station_type=station.station_type,
            phases=phases,
            current_phase=DockingPhase.APPROACH,
            phase_start_time=time.time(),
            approach_point=station.approach_point,
            dock_point=station.position,
            dock_heading=station.orientation,
            turret_target=station.turret_orientation if hasattr(station, 'turret_orientation') else 0.0,
            docking_speed=0.1,
            alignment_tolerance=self.alignment_tolerance,
            position_tolerance=self.position_tolerance,
            operation_duration=operation_duration,
            operation_start_time=0.0,
            charge_start_battery=robot.battery.soc if hasattr(robot, 'battery') else 0.0,
        )

        return sequence

    def update(self, robot, sequence: DockingSequence, dt: float) -> DockingPhase:
        """
        Update the docking state machine for one time step.

        Transitions between phases based on achievement of phase objectives.
        Handles all phase-specific behaviors including charging, payload ops, etc.

        Args:
            robot: AMRRobot instance (position, heading, turret_heading, velocity methods)
            sequence: Current DockingSequence
            dt: Time step in seconds

        Returns:
            Current DockingPhase after update
        """
        current_time = time.time()
        time_in_phase = current_time - sequence.phase_start_time

        if sequence.current_phase == DockingPhase.APPROACH:
            # Navigate to approach point
            dist_to_approach = math.sqrt(
                (robot.position[0] - sequence.approach_point[0]) ** 2 +
                (robot.position[1] - sequence.approach_point[1]) ** 2
            )

            if dist_to_approach < self.APPROACH_DISTANCE_THRESHOLD:
                # Transition to alignment
                sequence.current_phase = DockingPhase.ALIGNMENT
                sequence.phase_start_time = current_time

        elif sequence.current_phase == DockingPhase.ALIGNMENT:
            # Rotate base to match dock heading
            heading_error = self._normalize_angle(sequence.dock_heading - robot.heading)

            if abs(heading_error) < self.alignment_tolerance:
                # Aligned, transition to next phase
                next_phase_idx = sequence.phases.index(DockingPhase.ALIGNMENT) + 1
                sequence.current_phase = sequence.phases[next_phase_idx]
                sequence.phase_start_time = current_time
            else:
                # Apply P-controller
                robot.set_angular_velocity(
                    self._clamp(
                        self.BASE_ALIGNMENT_KP * heading_error,
                        -self.MAX_ANGULAR_VELOCITY,
                        self.MAX_ANGULAR_VELOCITY
                    )
                )

        elif sequence.current_phase == DockingPhase.TURRET_ALIGN:
            # Rotate turret to load transfer orientation
            turret_error = self._normalize_angle(sequence.turret_target - robot.turret_heading)

            if abs(turret_error) < self.alignment_tolerance:
                # Aligned, transition to final approach
                next_phase_idx = sequence.phases.index(DockingPhase.TURRET_ALIGN) + 1
                sequence.current_phase = sequence.phases[next_phase_idx]
                sequence.phase_start_time = current_time
            else:
                # Apply P-controller
                robot.rotate_turret(
                    self._clamp(
                        self.TURRET_ALIGNMENT_KP * turret_error,
                        -self.MAX_TURRET_VELOCITY,
                        self.MAX_TURRET_VELOCITY
                    )
                )

        elif sequence.current_phase == DockingPhase.FINAL_APPROACH:
            # Creep forward to dock point with heading correction
            dist_to_dock = math.sqrt(
                (robot.position[0] - sequence.dock_point[0]) ** 2 +
                (robot.position[1] - sequence.dock_point[1]) ** 2
            )

            if dist_to_dock < self.position_tolerance:
                # Reached dock point
                sequence.current_phase = DockingPhase.DOCKED
                sequence.phase_start_time = current_time
                sequence.operation_start_time = current_time
                robot.set_velocity(0.0, 0.0)
            else:
                # Move forward with heading correction
                heading_error = self._normalize_angle(sequence.dock_heading - robot.heading)
                angular_correction = self._clamp(
                    self.BASE_ALIGNMENT_KP * heading_error,
                    -self.MAX_ANGULAR_VELOCITY / 10.0,
                    self.MAX_ANGULAR_VELOCITY / 10.0
                )
                robot.set_velocity(sequence.docking_speed, angular_correction)

        elif sequence.current_phase == DockingPhase.DOCKED:
            # Perform docking operation
            if sequence.operation_duration == 0.0:
                # Parking bay (no operation)
                sequence.current_phase = DockingPhase.COMPLETE
            else:
                operation_elapsed = current_time - sequence.operation_start_time

                # Station-specific operations
                if sequence.station_type == StationType.CHARGING_STATION:
                    # Charge the battery
                    if hasattr(robot, 'battery'):
                        robot.battery.charge(dt)
                        # Transition when fully charged or enough time elapsed
                        if robot.battery.is_full() or operation_elapsed >= sequence.operation_duration:
                            sequence.current_phase = DockingPhase.UNDOCKING
                            sequence.phase_start_time = current_time
                    else:
                        # Fallback: just wait
                        if operation_elapsed >= sequence.operation_duration:
                            sequence.current_phase = DockingPhase.UNDOCKING
                            sequence.phase_start_time = current_time

                elif sequence.station_type == StationType.LOADING_DOCK:
                    # Wait 15 seconds then load payload
                    if operation_elapsed >= sequence.operation_duration:
                        if hasattr(robot, 'load_payload'):
                            robot.load_payload(100.0)  # 100kg default
                        sequence.current_phase = DockingPhase.UNDOCKING
                        sequence.phase_start_time = current_time

                elif sequence.station_type == StationType.UNLOADING_DOCK:
                    # Wait 10 seconds then unload payload
                    if operation_elapsed >= sequence.operation_duration:
                        if hasattr(robot, 'unload_payload'):
                            robot.unload_payload()
                        sequence.current_phase = DockingPhase.UNDOCKING
                        sequence.phase_start_time = current_time

            # Stop movement during docking
            robot.set_velocity(0.0, 0.0)

        elif sequence.current_phase == DockingPhase.UNDOCKING:
            # Back away from dock
            dist_backed = time_in_phase * sequence.docking_speed

            if dist_backed >= self.UNDOCKING_DISTANCE:
                # Backed away sufficiently
                sequence.current_phase = DockingPhase.COMPLETE
                robot.set_velocity(0.0, 0.0)
            else:
                # Reverse
                robot.set_velocity(-sequence.docking_speed, 0.0)

        elif sequence.current_phase == DockingPhase.COMPLETE:
            # Docking complete, stop all motion
            robot.set_velocity(0.0, 0.0)

        return sequence.current_phase

    def get_velocity_command(self, robot, sequence: DockingSequence) -> Tuple[float, float, float]:
        """
        Compute velocity commands for the current docking phase.

        Args:
            robot: AMRRobot instance
            sequence: Current DockingSequence

        Returns:
            Tuple of (linear_velocity, angular_velocity, turret_angular_velocity)
        """
        if sequence.current_phase == DockingPhase.APPROACH:
            # Path planner handles this
            return (0.0, 0.0, 0.0)

        elif sequence.current_phase == DockingPhase.ALIGNMENT:
            heading_error = self._normalize_angle(sequence.dock_heading - robot.heading)
            angular_vel = self._clamp(
                self.BASE_ALIGNMENT_KP * heading_error,
                -self.MAX_ANGULAR_VELOCITY,
                self.MAX_ANGULAR_VELOCITY
            )
            return (0.0, angular_vel, 0.0)

        elif sequence.current_phase == DockingPhase.TURRET_ALIGN:
            turret_error = self._normalize_angle(sequence.turret_target - robot.turret_heading)
            turret_vel = self._clamp(
                self.TURRET_ALIGNMENT_KP * turret_error,
                -self.MAX_TURRET_VELOCITY,
                self.MAX_TURRET_VELOCITY
            )
            return (0.0, 0.0, turret_vel)

        elif sequence.current_phase == DockingPhase.FINAL_APPROACH:
            heading_error = self._normalize_angle(sequence.dock_heading - robot.heading)
            angular_correction = self._clamp(
                self.BASE_ALIGNMENT_KP * heading_error,
                -self.MAX_ANGULAR_VELOCITY / 10.0,
                self.MAX_ANGULAR_VELOCITY / 10.0
            )
            return (sequence.docking_speed, angular_correction, 0.0)

        elif sequence.current_phase == DockingPhase.DOCKED:
            return (0.0, 0.0, 0.0)

        elif sequence.current_phase == DockingPhase.UNDOCKING:
            return (-sequence.docking_speed, 0.0, 0.0)

        else:  # COMPLETE
            return (0.0, 0.0, 0.0)

    def is_complete(self, sequence: DockingSequence) -> bool:
        """
        Check if docking sequence has completed.

        Args:
            sequence: DockingSequence to check

        Returns:
            True if sequence reached COMPLETE phase
        """
        return sequence.current_phase == DockingPhase.COMPLETE

    def get_progress(self, sequence: DockingSequence) -> Dict:
        """
        Get progress information for the docking sequence.

        Args:
            sequence: Current DockingSequence

        Returns:
            Dictionary with progress metrics
        """
        current_time = time.time()
        time_in_phase = current_time - sequence.phase_start_time

        # Estimate remaining time (rough estimate)
        phase_idx = sequence.phases.index(sequence.current_phase) if sequence.current_phase in sequence.phases else 0
        remaining_phases = len(sequence.phases) - phase_idx - 1
        estimated_remaining = sequence.operation_duration + (remaining_phases * 5.0)

        return {
            "current_phase": sequence.current_phase.value,
            "phase_index": phase_idx,
            "total_phases": len(sequence.phases),
            "time_in_phase": time_in_phase,
            "estimated_remaining_time": estimated_remaining,
            "is_complete": self.is_complete(sequence),
        }

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-pi, pi] range.

        Args:
            angle: Angle in radians

        Returns:
            Normalized angle
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """
        Clamp value to range.

        Args:
            value: Value to clamp
            min_val: Minimum
            max_val: Maximum

        Returns:
            Clamped value
        """
        return max(min_val, min(max_val, value))
