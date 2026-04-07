"""
Alert generation and management system for fleet coordination.

Generates structured alert messages when robots detect obstacles or enter
emergency states. Tracks alert history and enables fleet-wide notifications
to support coordinated response.

Author: Muskaan Maheshwari
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

from ..perception.obstacle_detector import ObstacleType, ObstacleSeverity


class AlertEventType(Enum):
    """Types of events that generate alerts."""
    OBSTACLE_DETECTED = "obstacle_detected"
    EMERGENCY_STOP = "emergency_stop"
    OBSTACLE_CLEARED = "obstacle_cleared"
    NAVIGATION_BLOCKED = "navigation_blocked"
    CRITICAL_BATTERY = "critical_battery"


class AlertDecision(Enum):
    """Possible decisions from alert processing."""
    STOP = "stop"
    REDIRECT = "redirect"
    WAIT = "wait"
    CONTINUE = "continue"


@dataclass
class AlertMessage:
    """
    Structured alert message for fleet coordination.

    Attributes:
        robot_id: Identifier of the robot generating the alert.
        timestamp: Unix timestamp of alert generation.
        event_type: Type of event from AlertEventType enum.
        location: Dictionary with 'x' and 'y' coordinates in meters.
        obstacle_distance: Distance to obstacle in meters.
        obstacle_type: Type of obstacle ('person', 'equipment', 'clutter', or None).
        robot_state: Current operational state of the robot.
        action_taken: Description of immediate action taken by robot.
        hub_response_required: Whether fleet hub intervention is needed.
        alert_id: Unique identifier for this alert.
        severity: Severity level ('high', 'medium', 'low') or None.
        confidence: Detection confidence [0, 1] if applicable.
    """

    robot_id: str
    timestamp: float
    event_type: AlertEventType
    location: Dict[str, float]
    obstacle_distance: float
    obstacle_type: Optional[str]
    robot_state: str
    action_taken: str
    hub_response_required: bool
    alert_id: str = ""
    severity: Optional[str] = None
    confidence: float = 1.0

    def to_dict(self) -> Dict:
        """
        Convert alert to JSON-serializable dictionary.

        Returns:
            Dictionary representation of the alert.
        """
        return {
            "robot_id": self.robot_id,
            "timestamp": self.timestamp,
            "event_type": self.event_type.value,
            "location": self.location,
            "obstacle_distance": self.obstacle_distance,
            "obstacle_type": self.obstacle_type,
            "robot_state": self.robot_state,
            "action_taken": self.action_taken,
            "hub_response_required": self.hub_response_required,
            "alert_id": self.alert_id,
            "severity": self.severity,
            "confidence": self.confidence,
        }


class AlertGenerator:
    """
    Creates structured alert messages for obstacle and emergency events.

    Generates unique alert IDs and formats messages for transmission to the
    fleet coordination hub.

    Attributes:
        alert_counter: Counter for generating unique alert IDs.
    """

    def __init__(self) -> None:
        """Initialize alert generator."""
        self.alert_counter = 0

    def create_obstacle_alert(
        self,
        robot_id: str,
        position: Tuple[float, float],
        obstacle_type: ObstacleType,
        distance: float,
        current_state: str,
        severity: ObstacleSeverity = ObstacleSeverity.MEDIUM,
        confidence: float = 0.8,
    ) -> AlertMessage:
        """
        Create an obstacle detection alert.

        Args:
            robot_id: Identifier of the detecting robot.
            position: Obstacle position as (x, y) in meters.
            obstacle_type: Type of obstacle from ObstacleType enum.
            distance: Distance to obstacle in meters.
            current_state: Current robot operational state (e.g., 'navigating', 'emergency_stop').
            severity: Obstacle severity from ObstacleSeverity enum.
            confidence: Detection confidence in [0, 1].

        Returns:
            AlertMessage object ready for transmission.

        Example:
            alert = generator.create_obstacle_alert(
                robot_id='amr_001',
                position=(5.2, 3.1),
                obstacle_type=ObstacleType.PERSON,
                distance=0.8,
                current_state='navigating',
                severity=ObstacleSeverity.HIGH
            )
            message = alert.to_dict()
        """
        self.alert_counter += 1
        alert_id = f"alert_{self.alert_counter:06d}"

        # Determine if hub response is needed
        hub_response_required = (
            distance < 0.5 or
            severity == ObstacleSeverity.HIGH or
            obstacle_type == ObstacleType.PERSON
        )

        # Determine action taken
        if distance < 0.3:
            action_taken = "emergency_stop_activated"
        elif distance < 0.5:
            action_taken = "dwa_avoidance_engaged"
        else:
            action_taken = "monitoring_active"

        alert = AlertMessage(
            robot_id=robot_id,
            timestamp=time.time(),
            event_type=AlertEventType.OBSTACLE_DETECTED,
            location={"x": position[0], "y": position[1]},
            obstacle_distance=distance,
            obstacle_type=obstacle_type.value,
            robot_state=current_state,
            action_taken=action_taken,
            hub_response_required=hub_response_required,
            alert_id=alert_id,
            severity=severity.value,
            confidence=confidence,
        )

        return alert

    def create_emergency_stop_alert(
        self,
        robot_id: str,
        position: Tuple[float, float],
        reason: str,
    ) -> AlertMessage:
        """
        Create an emergency stop alert.

        Args:
            robot_id: Identifier of the robot.
            position: Robot position as (x, y) in meters.
            reason: Reason for emergency stop (e.g., "obstacle_too_close").

        Returns:
            AlertMessage object.
        """
        self.alert_counter += 1
        alert_id = f"alert_{self.alert_counter:06d}"

        alert = AlertMessage(
            robot_id=robot_id,
            timestamp=time.time(),
            event_type=AlertEventType.EMERGENCY_STOP,
            location={"x": position[0], "y": position[1]},
            obstacle_distance=0.0,
            obstacle_type=None,
            robot_state="emergency_stop",
            action_taken="all_motion_halted",
            hub_response_required=True,
            alert_id=alert_id,
            severity="high",
            confidence=1.0,
        )

        return alert

    def create_cleared_alert(
        self,
        robot_id: str,
        position: Tuple[float, float],
    ) -> AlertMessage:
        """
        Create an obstacle cleared alert.

        Args:
            robot_id: Identifier of the robot.
            position: Robot position as (x, y) in meters.

        Returns:
            AlertMessage object.
        """
        self.alert_counter += 1
        alert_id = f"alert_{self.alert_counter:06d}"

        alert = AlertMessage(
            robot_id=robot_id,
            timestamp=time.time(),
            event_type=AlertEventType.OBSTACLE_CLEARED,
            location={"x": position[0], "y": position[1]},
            obstacle_distance=float("inf"),
            obstacle_type=None,
            robot_state="navigating",
            action_taken="resuming_path",
            hub_response_required=False,
            alert_id=alert_id,
            severity="low",
            confidence=1.0,
        )

        return alert


class AlertHub:
    """
    Central hub for receiving and processing robot alerts.

    Maintains alert history and makes coordinated decisions for fleet response
    to obstacles and emergencies.

    Attributes:
        max_history_size: Maximum number of alerts to retain in history.
        alert_timeout: Time in seconds before alerts expire from active status.
    """

    def __init__(self, max_history_size: int = 1000, alert_timeout: float = 30.0) -> None:
        """
        Initialize alert hub.

        Args:
            max_history_size: Maximum alerts in history (default 1000).
            alert_timeout: Alert timeout in seconds (default 30.0).
        """
        self.max_history_size = max_history_size
        self.alert_timeout = alert_timeout

        # Alert tracking
        self.alert_history: List[AlertMessage] = []
        self.active_alerts: Dict[str, AlertMessage] = {}  # robot_id -> most recent alert

    def process_alert(
        self,
        alert: AlertMessage,
    ) -> AlertDecision:
        """
        Process incoming alert and make coordinated response decision.

        Args:
            alert: AlertMessage to process.

        Returns:
            AlertDecision indicating recommended action for fleet.

        Example:
            decision = hub.process_alert(alert)
            if decision == AlertDecision.STOP:
                # Pause all nearby robots
                pass
            elif decision == AlertDecision.REDIRECT:
                # Find alternate routes for affected robots
                pass
        """
        # Record alert in history
        self.alert_history.append(alert)

        # Trim history if too large
        if len(self.alert_history) > self.max_history_size:
            self.alert_history = self.alert_history[-self.max_history_size:]

        # Track active alert for this robot
        self.active_alerts[alert.robot_id] = alert

        # Make decision based on alert characteristics
        decision = self._make_decision(alert)

        return decision

    def get_active_alerts(self) -> Dict[str, AlertMessage]:
        """
        Get currently active alerts, removing expired ones.

        Returns:
            Dictionary mapping robot_id to most recent active alert.
        """
        current_time = time.time()
        expired_robots = []

        for robot_id, alert in self.active_alerts.items():
            if current_time - alert.timestamp > self.alert_timeout:
                expired_robots.append(robot_id)

        for robot_id in expired_robots:
            del self.active_alerts[robot_id]

        return self.active_alerts

    def get_alert_history(
        self,
        robot_id: Optional[str] = None,
        event_type: Optional[AlertEventType] = None,
        limit: int = 100,
    ) -> List[AlertMessage]:
        """
        Retrieve alert history with optional filtering.

        Args:
            robot_id: Filter by robot ID (None = all robots).
            event_type: Filter by event type (None = all types).
            limit: Maximum number of alerts to return (default 100).

        Returns:
            Filtered list of alerts from history.
        """
        filtered = self.alert_history

        if robot_id is not None:
            filtered = [a for a in filtered if a.robot_id == robot_id]

        if event_type is not None:
            filtered = [a for a in filtered if a.event_type == event_type]

        # Return most recent first
        return filtered[-limit:][::-1]

    def _make_decision(self, alert: AlertMessage) -> AlertDecision:
        """
        Make fleet response decision based on alert characteristics.

        Args:
            alert: AlertMessage to evaluate.

        Returns:
            Recommended fleet action.
        """
        # High severity or close distance -> stop
        if alert.severity == "high" and alert.obstacle_distance < 1.0:
            return AlertDecision.STOP

        # Person obstacle -> stop
        if alert.obstacle_type == "person":
            return AlertDecision.STOP

        # Emergency stop -> stop
        if alert.event_type == AlertEventType.EMERGENCY_STOP:
            return AlertDecision.STOP

        # Equipment or clutter with medium severity -> redirect
        if alert.severity == "medium" and alert.obstacle_distance < 3.0:
            return AlertDecision.REDIRECT

        # Low severity and reasonable distance -> wait
        if alert.severity == "low" and alert.obstacle_distance > 2.0:
            return AlertDecision.WAIT

        # Default to continue monitoring
        return AlertDecision.CONTINUE

    def clear_alert(self, robot_id: str) -> bool:
        """
        Mark alert for a robot as cleared.

        Args:
            robot_id: Robot whose alert to clear.

        Returns:
            True if alert was cleared, False if no active alert for robot.
        """
        if robot_id in self.active_alerts:
            del self.active_alerts[robot_id]
            return True
        return False

    def get_statistics(self) -> Dict:
        """
        Get alert statistics from current session.

        Returns:
            Dictionary with alert counts and summaries.
        """
        stats = {
            "total_alerts": len(self.alert_history),
            "active_alerts": len(self.active_alerts),
            "alerts_by_type": {},
            "alerts_by_robot": {},
        }

        # Count by event type
        for alert in self.alert_history:
            event_key = alert.event_type.value
            stats["alerts_by_type"][event_key] = stats["alerts_by_type"].get(event_key, 0) + 1

            # Count by robot
            robot_key = alert.robot_id
            stats["alerts_by_robot"][robot_key] = stats["alerts_by_robot"].get(robot_key, 0) + 1

        return stats
