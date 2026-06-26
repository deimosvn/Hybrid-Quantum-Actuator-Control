# Copyright 2026 Diego Eduardo Martinez Cruz
# SPDX-License-Identifier: MIT
"""Full-system bring-up: plant simulator + hybrid controller + logger + RViz.

The controller is a lifecycle node; this launch file configures and activates
it automatically through the launch event system, so ``ros2 launch`` yields a
running, visualized closed loop in one command::

    ros2 launch quantum_rover_bringup quantum_rover.launch.py
    ros2 launch quantum_rover_bringup quantum_rover.launch.py use_quantum:=false
"""

import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    control_share = FindPackageShare('quantum_rover_control')
    description_share = FindPackageShare('quantum_rover_description')

    default_params = PathJoinSubstitution(
        [control_share, 'config', 'control_params.yaml'])

    use_quantum = LaunchConfiguration('use_quantum')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')
    auto_activate = LaunchConfiguration('auto_activate')

    args = [
        DeclareLaunchArgument('use_quantum', default_value='true',
                              description='Start the controller in QAOA mode.'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Open RViz2.'),
        DeclareLaunchArgument('auto_activate', default_value='true',
                              description='Auto configure+activate the controller.'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML parameters for all nodes.'),
    ]

    simulator = Node(
        package='quantum_rover_control',
        executable='rover_simulator',
        name='rover_simulator',
        output='screen',
        parameters=[params_file],
    )

    controller = LifecycleNode(
        package='quantum_rover_control',
        executable='quantum_controller',
        name='quantum_controller',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_quantum': use_quantum}],
    )

    logger = Node(
        package='quantum_rover_control',
        executable='telemetry_logger',
        name='telemetry_logger',
        output='screen',
        parameters=[params_file],
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([description_share, 'launch', 'description.launch.py'])),
        launch_arguments={'use_rviz': use_rviz, 'use_jsp_gui': 'false'}.items(),
    )

    # --- Lifecycle orchestration ---------------------------------------
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(controller),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(auto_activate),
    )

    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=controller,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(controller),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))],
        ),
        condition=IfCondition(auto_activate),
    )

    return LaunchDescription(
        args + [simulator, controller, logger, description,
                activate_on_inactive, configure_event])
