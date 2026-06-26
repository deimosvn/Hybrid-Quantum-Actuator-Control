#!/usr/bin/env python3
# Copyright 2026 Diego Eduardo Martinez Cruz
# SPDX-License-Identifier: MIT
"""Rover plant simulator node.

Integrates the DC-motor + traction-loss dynamics from
:mod:`quantum_rover_control.core` and republishes the result on standard ROS 2
interfaces, so the rest of the stack (controller, RViz, ``ros2 bag``) sees a
realistic 1-DOF differential drive.

Subscribes
----------
``~/motor_cmd``    : :class:`std_msgs.msg.Float64` -- commanded motor voltage (V)

Publishes
---------
``~/motor_state``  : :class:`quantum_rover_interfaces.msg.MotorState`
``joint_states``   : :class:`sensor_msgs.msg.JointState`
``odom``           : :class:`nav_msgs.msg.Odometry`
TF                 : ``odom`` -> ``base_link``
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from quantum_rover_control.core import (
    RoverDynamics, MotorParameters, RoverParameters,
)
from quantum_rover_interfaces.msg import MotorState


def _float_param(description: str, lower: float, upper: float) -> ParameterDescriptor:
    """Build a float ParameterDescriptor with an inclusive range constraint."""
    return ParameterDescriptor(
        description=description,
        floating_point_range=[FloatingPointRange(from_value=lower, to_value=upper, step=0.0)],
    )


class RoverSimulatorNode(Node):
    """Forward-integrates the plant and publishes its observable state."""

    def __init__(self) -> None:
        super().__init__('rover_simulator')

        # --- Parameters -----------------------------------------------------
        self.declare_parameter(
            'sim_frequency', 1000,
            ParameterDescriptor(
                description='Physics integration rate (Hz).',
                integer_range=[IntegerRange(from_value=100, to_value=10000, step=0)]))
        self.declare_parameter(
            'publish_frequency', 100.0,
            _float_param('Rate at which state is published to the graph (Hz).', 1.0, 1000.0))

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter(
            'wheel_joints',
            ['front_left_wheel_joint', 'front_right_wheel_joint',
             'rear_left_wheel_joint', 'rear_right_wheel_joint'],
            ParameterDescriptor(description='Names of the drive-wheel joints to publish.'))

        # Motor electromechanical parameters.
        self.declare_parameter('motor.K_t', 0.15,
                               _float_param('Torque constant (N*m/A).', 0.0, 5.0))
        self.declare_parameter('motor.K_e', 0.15,
                               _float_param('Back-EMF constant (V*s/rad).', 0.0, 5.0))
        self.declare_parameter('motor.R', 1.5,
                               _float_param('Armature resistance (Ohm).', 0.01, 100.0))
        self.declare_parameter('motor.J', 0.01,
                               _float_param('Rotor inertia (kg*m^2).', 1e-5, 10.0))
        self.declare_parameter('motor.b', 0.03,
                               _float_param('Viscous damping (N*m*s/rad).', 0.0, 5.0))
        self.declare_parameter('motor.V_max', 12.0,
                               _float_param('Maximum terminal voltage (V).', 1.0, 60.0))
        self.declare_parameter('motor.friction_coefficient', 0.15,
                               _float_param('Static friction coefficient.', 0.0, 1.0))

        # Rover body parameters.
        self.declare_parameter('rover.mass', 5.0,
                               _float_param('Total rover mass (kg).', 0.1, 1000.0))
        self.declare_parameter('rover.wheel_radius', 0.1,
                               _float_param('Drive-wheel radius (m).', 0.01, 2.0))
        self.declare_parameter('rover.num_wheels', 4.0,
                               _float_param('Number of drive wheels.', 1.0, 12.0))
        self.declare_parameter('rover.traction_loss', 0.2,
                               _float_param('Fraction of traction lost to slip [0, 1].', 0.0, 1.0))

        p = self.get_parameter
        self._odom_frame = p('odom_frame').value
        self._base_frame = p('base_frame').value
        self._wheel_joints = list(p('wheel_joints').value)

        self._motor_params = MotorParameters(
            K_t=p('motor.K_t').value,
            K_e=p('motor.K_e').value,
            R=p('motor.R').value,
            J=p('motor.J').value,
            b=p('motor.b').value,
            V_max=p('motor.V_max').value,
            friction_coefficient=p('motor.friction_coefficient').value,
        )
        self._rover_params = RoverParameters(
            mass=p('rover.mass').value,
            wheel_radius=p('rover.wheel_radius').value,
            num_wheels=p('rover.num_wheels').value,
            traction_loss=p('rover.traction_loss').value,
        )

        # --- Plant ----------------------------------------------------------
        self._sim_freq = int(p('sim_frequency').value)
        self._pub_freq = float(p('publish_frequency').value)
        self._dt = 1.0 / self._sim_freq
        self._substeps = max(1, int(round(self._sim_freq / self._pub_freq)))

        self._rover = RoverDynamics(self._motor_params, self._rover_params)
        # Align the integrator step with the configured simulation frequency.
        self._rover.dt = self._dt
        self._rover.motor.dt = self._dt

        self._cmd_voltage = 0.0
        self._wheel_angle = 0.0

        # --- ROS interfaces -------------------------------------------------
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self._cmd_sub = self.create_subscription(
            Float64, 'motor_cmd', self._on_motor_cmd, control_qos)

        self._motor_state_pub = self.create_publisher(MotorState, 'motor_state', control_qos)
        self._joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._odom_pub = self.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._timer = self.create_timer(1.0 / self._pub_freq, self._on_timer)

        self.get_logger().info(
            f"Rover simulator up: integrating @ {self._sim_freq} Hz "
            f"({self._substeps} substeps/cycle), publishing @ {self._pub_freq} Hz.")

    # ------------------------------------------------------------------ #
    def _on_motor_cmd(self, msg: Float64) -> None:
        self._cmd_voltage = float(msg.data)

    def _on_timer(self) -> None:
        """Advance the plant by one publish period and emit observable state."""
        state = {}
        for _ in range(self._substeps):
            state = self._rover.step(self._cmd_voltage)
        if not state:
            return

        now = self.get_clock().now().to_msg()
        omega = state['omega']
        self._wheel_angle += omega * (self._substeps * self._dt)

        # Recover electrical quantities from the kinematic state.
        current = (self._cmd_voltage - self._motor_params.K_e * omega) / self._motor_params.R
        torque = self._motor_params.K_t * current

        # MotorState --------------------------------------------------------
        ms = MotorState()
        ms.header.stamp = now
        ms.header.frame_id = self._base_frame
        ms.omega = omega
        ms.domega_dt = state['domega_dt']
        ms.current = float(current)
        ms.torque = float(torque)
        ms.voltage = float(self._cmd_voltage)
        self._motor_state_pub.publish(ms)

        # JointState --------------------------------------------------------
        # Single-DOF drive: every wheel turns at the same rate.
        n = len(self._wheel_joints)
        js = JointState()
        js.header.stamp = now
        js.name = list(self._wheel_joints)
        js.position = [self._wheel_angle] * n
        js.velocity = [omega] * n
        js.effort = [float(torque)] * n
        self._joint_pub.publish(js)

        # Odometry ----------------------------------------------------------
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = state['position']
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = state['velocity']
        self._odom_pub.publish(odom)

        # TF odom -> base_link ---------------------------------------------
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = state['position']
        tf.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoverSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
