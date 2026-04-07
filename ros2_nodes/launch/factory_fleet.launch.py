"""
ROS2 Launch file for complete multi-AMR factory system.

Launches all nodes needed for the factory fleet:
  - Fleet manager (central orchestrator)
  - Path planner service
  - Traffic controller
  - N x AMR driver nodes (one per robot)
  - Gazebo simulator (optional, for visualization)

Usage:
    ros2 launch ros2_nodes factory_fleet.launch.py num_robots:=5 gazebo:=true

Author: Muskaan Maheshwari
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate launch configuration for factory fleet system."""

    # Declare launch arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='3',
        description='Number of AMR robots in the fleet'
    )

    use_gazebo_arg = DeclareLaunchArgument(
        'gazebo',
        default_value='false',
        description='Launch Gazebo simulator for visualization'
    )

    start_x_arg = DeclareLaunchArgument(
        'start_x',
        default_value='0.0',
        description='Initial X position for first robot'
    )

    start_y_arg = DeclareLaunchArgument(
        'start_y',
        default_value='0.0',
        description='Initial Y position for first robot'
    )

    robot_spacing_arg = DeclareLaunchArgument(
        'robot_spacing',
        default_value='5.0',
        description='Spacing between robot initial positions (meters)'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10',
        description='Update rate for robot drivers (Hz)'
    )

    # Retrieve launch configuration values
    num_robots = LaunchConfiguration('num_robots')
    use_gazebo = LaunchConfiguration('gazebo')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    robot_spacing = LaunchConfiguration('robot_spacing')
    update_rate = LaunchConfiguration('update_rate')

    # Static nodes
    fleet_manager_node = Node(
        package='ros2_multi_amr_factory',
        executable='fleet_manager',
        name='fleet_manager',
        output='screen',
        parameters=[
            {'update_rate': update_rate},
            {'battery_charge_threshold': 30.0},
            {'battery_target_charge': 80.0},
        ]
    )

    path_planner_node = Node(
        package='ros2_multi_amr_factory',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[
            {'turning_radius': 2.0},
            {'factory_grid_size': 30.0},
        ]
    )

    traffic_controller_node = Node(
        package='ros2_multi_amr_factory',
        executable='traffic_controller',
        name='traffic_controller',
        output='screen',
        parameters=[
            {'num_intersections': 9},
            {'intersection_radius': 3.0},
            {'safety_radius': 1.5},
        ]
    )

    # Optional Gazebo simulator
    gazebo_node = Node(
        package='gazebo_ros',
        executable='gazebo',
        name='gazebo',
        output='screen',
        condition=lambda context: use_gazebo == 'true'
    )

    # RViz visualization
    rviz_config_path = os.path.join(
        os.path.dirname(__file__), '..', '..', 'config', 'factory_viz.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=lambda context: use_gazebo == 'true'
    )

    # Generate AMR driver nodes dynamically
    amr_nodes = []
    for i in range(int(num_robots.source.default_value) if hasattr(num_robots.source, 'default_value') else 3):
        robot_id = f'amr_{i+1:03d}'
        x_offset = i * 5.0  # Default spacing

        amr_node = Node(
            package='ros2_multi_amr_factory',
            executable='amr_driver',
            name=robot_id,
            output='screen',
            parameters=[
                {'robot_id': robot_id},
                {'start_x': x_offset},
                {'start_y': 0.0},
                {'start_heading': 0.0},
                {'update_rate': update_rate},
            ],
            remappings=[
                (f'/{robot_id}/cmd_vel', f'/{robot_id}/cmd_vel'),
                (f'/{robot_id}/odom', f'/{robot_id}/odom'),
            ]
        )
        amr_nodes.append(amr_node)

    # Assemble launch description
    launch_description = LaunchDescription([
        # Declare launch arguments
        num_robots_arg,
        use_gazebo_arg,
        start_x_arg,
        start_y_arg,
        robot_spacing_arg,
        update_rate_arg,
        # Static nodes
        fleet_manager_node,
        path_planner_node,
        traffic_controller_node,
        # Visualization (optional)
        rviz_node,
        gazebo_node,
    ])

    # Add dynamic AMR nodes
    for amr_node in amr_nodes:
        launch_description.add_action(amr_node)

    return launch_description
