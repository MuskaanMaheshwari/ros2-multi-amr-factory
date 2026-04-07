"""
Visualization Demos Module

Advanced visualization demonstrations for the ros2-multi-amr-factory system:
- Navigation: Dubins curve path planning with multi-frame snapshots
- Obstacle Avoidance: Dynamic Window Approach (DWA) in action
- Alert System: Multi-robot fleet coordination with alert broadcasting

Author: Muskaan Maheshwari
"""

from .navigation_demo import create_navigation_demo
from .obstacle_avoidance_demo import create_obstacle_avoidance_demo
from .alert_demo import create_alert_demo

__all__ = [
    'create_navigation_demo',
    'create_obstacle_avoidance_demo',
    'create_alert_demo',
]
