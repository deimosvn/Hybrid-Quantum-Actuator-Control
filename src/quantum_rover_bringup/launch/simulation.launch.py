# Copyright 2026 Diego Eduardo Martinez Cruz
# SPDX-License-Identifier: MIT
"""Headless closed-loop simulation (no RViz) for benchmarking / CI.

Brings up the plant simulator, the auto-activated hybrid controller and the
telemetry logger. Select the backend with ``use_quantum``::

    ros2 launch quantum_rover_bringup simulation.launch.py use_quantum:=true
    ros2 launch quantum_rover_bringup simulation.launch.py use_quantum:=false
"""

import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    control_share = FindPackageShare('quantum_rover_control')
    default_params = PathJoinSubstitution(
        [control_share, 'config', 'control_params.yaml'])

    use_quantum = LaunchConfiguration('use_quantum')
    params_file = LaunchConfiguration('params_file')

    args = [
        DeclareLaunchArgument('use_quantum', default_value='false',
                              description='QAOA (true) or PID (false).'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML parameters for all nodes.'),
    ]

    simulator = Node(
        package='quantum_rover_control', executable='rover_simulator',
        name='rover_simulator', output='screen', parameters=[params_file])

    controller = LifecycleNode(
        package='quantum_rover_control', executable='quantum_controller',
        name='quantum_controller', namespace='', output='screen',
        parameters=[params_file, {'use_quantum': use_quantum}])

    logger = Node(
        package='quantum_rover_control', executable='telemetry_logger',
        name='telemetry_logger', output='screen', parameters=[params_file])

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(controller),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))

    activate_on_inactive = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=controller,
        start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(controller),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription(
        args + [simulator, controller, logger, activate_on_inactive, configure_event])
