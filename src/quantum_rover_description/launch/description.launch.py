# Copyright 2026 Diego Eduardo Martinez Cruz
# SPDX-License-Identifier: MIT
"""Publish the rover's robot_description and (optionally) open RViz.

Standalone visualization (no simulator): launch with ``use_jsp_gui:=true`` to
get sliders for the wheel joints. When the rover_simulator node is running it
publishes /joint_states itself, so leave the GUI off.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('quantum_rover_description')

    xacro_path = PathJoinSubstitution([pkg, 'urdf', 'quantum_rover.urdf.xacro'])
    rviz_path = PathJoinSubstitution([pkg, 'rviz', 'quantum_rover.rviz'])

    robot_description = {
        'robot_description': Command(['xacro ', xacro_path]),
    }

    use_rviz = LaunchConfiguration('use_rviz')
    use_jsp_gui = LaunchConfiguration('use_jsp_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Open RViz2 with the rover view.'),
        DeclareLaunchArgument('use_jsp_gui', default_value='false',
                              description='Run joint_state_publisher_gui (no simulator).'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the /clock topic for time.'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_jsp_gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
