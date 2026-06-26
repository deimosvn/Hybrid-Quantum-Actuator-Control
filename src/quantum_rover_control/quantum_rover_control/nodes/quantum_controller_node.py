#!/usr/bin/env python3
# Copyright 2026 Diego Eduardo Martinez Cruz
# SPDX-License-Identifier: MIT
"""Hybrid quantum-classical controller node (managed / lifecycle).

Closes the velocity-tracking loop around the rover plant. Each control cycle it
reads the measured state, computes the tracking error, asks the active optimizer
(QAOA or PID fallback) for an action, and publishes a motor-voltage command.

Implemented as a **lifecycle node** so an orchestrator can configure/activate it
deterministically. While ``inactive`` no commands are emitted.

Subscribes
----------
``~/motor_state``      : :class:`quantum_rover_interfaces.msg.MotorState`
``~/odom``             : :class:`nav_msgs.msg.Odometry`
``~/reference_omega``  : :class:`std_msgs.msg.Float64` (rad/s)

Publishes
---------
``~/motor_cmd``        : :class:`std_msgs.msg.Float64` (V)
``~/telemetry``        : :class:`quantum_rover_interfaces.msg.RoverTelemetry`
``/diagnostics``       : :class:`diagnostic_msgs.msg.DiagnosticArray`

Services
--------
``~/set_control_mode`` : quantum_rover_interfaces/SetControlMode
``~/optimize_control`` : quantum_rover_interfaces/OptimizeControl

Action
------
``~/follow_velocity_profile`` : quantum_rover_interfaces/FollowVelocityProfile
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState as State
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from quantum_rover_control.core import QAOAController, FallbackController
from quantum_rover_interfaces.msg import MotorState, RoverTelemetry
from quantum_rover_interfaces.srv import SetControlMode, OptimizeControl
from quantum_rover_interfaces.action import FollowVelocityProfile


def _float_param(description: str, lower: float, upper: float) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=description,
        floating_point_range=[FloatingPointRange(from_value=lower, to_value=upper, step=0.0)])


class QuantumControllerNode(LifecycleNode):
    """Lifecycle node implementing the hybrid control loop."""

    def __init__(self) -> None:
        super().__init__('quantum_controller')

        # Parameters are safe to declare in the constructor; heavy resources
        # (optimizer, publishers, timer) are created during the lifecycle
        # transitions below.
        self.declare_parameter('use_quantum', True,
                               ParameterDescriptor(description='Start in QAOA mode (else PID).'))
        self.declare_parameter('num_qubits', 2, ParameterDescriptor(
            description='QAOA register width.',
            integer_range=[IntegerRange(from_value=1, to_value=8, step=0)]))
        self.declare_parameter('num_layers', 1, ParameterDescriptor(
            description='QAOA depth (p parameter).',
            integer_range=[IntegerRange(from_value=1, to_value=6, step=0)]))
        self.declare_parameter('control_frequency', 100.0,
                               _float_param('Control-loop rate (Hz).', 1.0, 1000.0))
        self.declare_parameter('pwm_max', 12.0,
                               _float_param('Voltage saturation (V).', 1.0, 60.0))
        self.declare_parameter('reference_omega', 5.0,
                               _float_param('Initial velocity setpoint (rad/s).', -50.0, 50.0))
        self.declare_parameter('reference_position', 0.0,
                               _float_param('Position setpoint (m).', -1e6, 1e6))
        self.declare_parameter('q_position_weight', 10.0,
                               _float_param('LQR state weight on position error.', 0.0, 1e4))
        self.declare_parameter('q_velocity_weight', 1.0,
                               _float_param('LQR state weight on velocity error.', 0.0, 1e4))
        self.declare_parameter('r_control_weight', 0.5,
                               _float_param('LQR weight on control effort.', 1e-6, 1e4))

        # Runtime state.
        self._optimizer = None
        self._control_mode = 'classical'
        self._measured_omega = 0.0
        self._measured_domega = 0.0
        self._position = 0.0
        self._reference_omega = 0.0
        self._last_control = 0.0
        self._iteration = 0
        self._t0 = None

        # Interfaces (populated in on_configure).
        self._cmd_pub = None
        self._telemetry_pub = None
        self._diag_pub = None
        self._control_timer = None

        self._reentrant_cb = ReentrantCallbackGroup()
        self._control_cb = MutuallyExclusiveCallbackGroup()

    # ================================================================== #
    #  Lifecycle transitions
    # ================================================================== #
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring quantum_controller...')

        self._reference_omega = float(self.get_parameter('reference_omega').value)
        requested = 'quantum' if self.get_parameter('use_quantum').value else 'classical'
        self._build_optimizer(requested)

        control_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 history=HistoryPolicy.KEEP_LAST, depth=10)

        # Lifecycle publishers: silent unless the node is active.
        self._cmd_pub = self.create_lifecycle_publisher(
            Float64, 'motor_cmd', control_qos)
        self._telemetry_pub = self.create_lifecycle_publisher(
            RoverTelemetry, 'telemetry', control_qos)
        self._diag_pub = self.create_lifecycle_publisher(
            DiagnosticArray, '/diagnostics', 10)

        self.create_subscription(MotorState, 'motor_state', self._on_motor_state, control_qos)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Float64, 'reference_omega', self._on_reference, control_qos)

        self.create_service(SetControlMode, '~/set_control_mode', self._on_set_mode)
        self.create_service(OptimizeControl, '~/optimize_control', self._on_optimize)

        self._action_server = ActionServer(
            self, FollowVelocityProfile, '~/follow_velocity_profile',
            execute_callback=self._execute_profile,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._reentrant_cb)

        self.get_logger().info(f'Configured in {self._control_mode.upper()} mode.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating control loop.')
        self._t0 = self.get_clock().now()
        self._iteration = 0
        period = 1.0 / float(self.get_parameter('control_frequency').value)
        self._control_timer = self.create_timer(
            period, self._control_step, callback_group=self._control_cb)
        # Activates the managed lifecycle publishers.
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating control loop.')
        if self._control_timer is not None:
            self.destroy_timer(self._control_timer)
            self._control_timer = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up.')
        self._optimizer = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down quantum_controller.')
        return TransitionCallbackReturn.SUCCESS

    # ================================================================== #
    #  Optimizer management
    # ================================================================== #
    def _cost_matrices(self):
        q = np.diag([float(self.get_parameter('q_position_weight').value),
                     float(self.get_parameter('q_velocity_weight').value)])
        r = np.array([[float(self.get_parameter('r_control_weight').value)]])
        return q, r

    def _build_optimizer(self, requested_mode: str) -> str:
        """Instantiate the requested optimizer, falling back to PID on failure."""
        q, r = self._cost_matrices()
        if requested_mode == 'quantum':
            try:
                opt = QAOAController(
                    num_qubits=int(self.get_parameter('num_qubits').value),
                    num_layers=int(self.get_parameter('num_layers').value),
                    use_simulator=True)
                self._control_mode = 'quantum' if opt.use_simulator else 'classical'
            except Exception as exc:  # noqa: BLE001 - degrade gracefully
                self.get_logger().warn(f'QAOA init failed ({exc}); falling back to PID.')
                opt = FallbackController()
                self._control_mode = 'classical'
        else:
            opt = FallbackController()
            self._control_mode = 'classical'

        opt.Q, opt.R = q, r
        self._optimizer = opt
        return self._control_mode

    # ================================================================== #
    #  Subscriptions
    # ================================================================== #
    def _on_motor_state(self, msg: MotorState) -> None:
        self._measured_omega = msg.omega
        self._measured_domega = msg.domega_dt

    def _on_odom(self, msg: Odometry) -> None:
        self._position = msg.pose.pose.position.x

    def _on_reference(self, msg: Float64) -> None:
        self._reference_omega = float(msg.data)

    # ================================================================== #
    #  Control loop
    # ================================================================== #
    def _control_step(self) -> None:
        if self._optimizer is None:
            return

        ref_pos = float(self.get_parameter('reference_position').value)
        pwm_max = float(self.get_parameter('pwm_max').value)

        position_error = ref_pos - self._position
        velocity_error = self._reference_omega - self._measured_omega
        error_state = np.array([position_error, velocity_error])

        t_start = time.perf_counter()
        control_norm = float(self._optimizer.optimize(error_state))
        opt_ms = (time.perf_counter() - t_start) * 1000.0

        voltage = float(np.clip(control_norm * pwm_max, -pwm_max, pwm_max))
        self._last_control = voltage

        cost = float(self._optimizer.cost_function(
            np.array([position_error, velocity_error, control_norm]),
            self._optimizer.Q, self._optimizer.R))

        self._cmd_pub.publish(Float64(data=voltage))

        now = self.get_clock().now()

        tel = RoverTelemetry()
        tel.header.stamp = now.to_msg()
        tel.header.frame_id = 'base_link'
        tel.iteration = self._iteration
        tel.position = self._position
        tel.velocity = self._measured_omega  # angular proxy for the 1-DOF drive
        tel.motor.header.stamp = tel.header.stamp
        tel.motor.omega = self._measured_omega
        tel.motor.domega_dt = self._measured_domega
        tel.motor.voltage = voltage
        tel.control_input = voltage
        tel.control_source = self._control_mode
        tel.position_error = position_error
        tel.velocity_error = velocity_error
        tel.cost_function = cost
        tel.optimization_time_ms = opt_ms
        tel.quantum_backend_ok = (self._control_mode == 'quantum')
        self._telemetry_pub.publish(tel)

        # Throttle diagnostics to ~2 Hz.
        freq = float(self.get_parameter('control_frequency').value)
        if self._iteration % max(1, int(freq / 2)) == 0:
            self._publish_diagnostics(velocity_error, opt_ms, cost)

        self._iteration += 1

    def _publish_diagnostics(self, velocity_error: float, opt_ms: float, cost: float) -> None:
        status = DiagnosticStatus()
        status.name = 'quantum_rover: hybrid_controller'
        status.hardware_id = 'quantum_rover'
        if abs(velocity_error) < 0.5:
            status.level = DiagnosticStatus.OK
            status.message = f'Tracking ({self._control_mode})'
        elif abs(velocity_error) < 3.0:
            status.level = DiagnosticStatus.WARN
            status.message = f'Settling ({self._control_mode})'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = f'Large tracking error ({self._control_mode})'
        status.values = [
            KeyValue(key='control_mode', value=self._control_mode),
            KeyValue(key='velocity_error', value=f'{velocity_error:.4f}'),
            KeyValue(key='reference_omega', value=f'{self._reference_omega:.4f}'),
            KeyValue(key='measured_omega', value=f'{self._measured_omega:.4f}'),
            KeyValue(key='cost_function', value=f'{cost:.4f}'),
            KeyValue(key='optimization_time_ms', value=f'{opt_ms:.3f}'),
            KeyValue(key='iteration', value=str(self._iteration)),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self._diag_pub.publish(arr)

    # ================================================================== #
    #  Services
    # ================================================================== #
    def _on_set_mode(self, request: SetControlMode.Request,
                     response: SetControlMode.Response) -> SetControlMode.Response:
        mode = request.mode.strip().lower()
        if mode not in ('quantum', 'classical'):
            response.success = False
            response.active_mode = self._control_mode
            response.message = f"Unknown mode '{request.mode}'. Use 'quantum' or 'classical'."
            return response
        active = self._build_optimizer(mode)
        response.success = True
        response.active_mode = active
        response.message = (f"Switched to {active}."
                            + (' (QAOA unavailable, using PID)'
                               if mode == 'quantum' and active == 'classical' else ''))
        self.get_logger().info(response.message)
        return response

    def _on_optimize(self, request: OptimizeControl.Request,
                     response: OptimizeControl.Response) -> OptimizeControl.Response:
        if self._optimizer is None:
            response.source = 'none'
            return response
        error_state = np.array([request.position_error, request.velocity_error])
        t_start = time.perf_counter()
        control_norm = float(self._optimizer.optimize(error_state))
        response.optimization_time_ms = (time.perf_counter() - t_start) * 1000.0
        response.control_normalized = control_norm
        response.cost = float(self._optimizer.cost_function(
            np.array([request.position_error, request.velocity_error, control_norm]),
            self._optimizer.Q, self._optimizer.R))
        response.source = self._control_mode
        return response

    # ================================================================== #
    #  Action: follow a velocity profile and report tracking metrics
    # ================================================================== #
    def _execute_profile(self, goal_handle) -> FollowVelocityProfile.Result:
        goal = goal_handle.request
        targets = list(goal.target_omega)
        holds = list(goal.hold_duration)
        tol = goal.settle_tolerance if goal.settle_tolerance > 0.0 else 0.25

        result = FollowVelocityProfile.Result()
        feedback = FollowVelocityProfile.Feedback()
        errors: list[float] = []
        controls: list[float] = []
        converged = True

        total_time = sum(holds) if holds else max(1.0, float(len(targets)))
        start = self.get_clock().now()
        sample_dt = 0.05

        self.get_logger().info(f'Velocity profile: {len(targets)} setpoint(s).')

        for i, target in enumerate(targets):
            hold = holds[i] if i < len(holds) else 1.0
            self._reference_omega = float(target)
            seg_start = self.get_clock().now()
            settled = False

            while (self.get_clock().now() - seg_start).nanoseconds * 1e-9 < hold:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.converged = False
                    self.get_logger().warn('Velocity profile canceled.')
                    return result

                err = float(target) - self._measured_omega
                errors.append(err)
                controls.append(self._last_control)
                if abs(err) <= tol:
                    settled = True

                elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
                feedback.current_omega = self._measured_omega
                feedback.target_omega = float(target)
                feedback.velocity_error = err
                feedback.elapsed_time = float(elapsed)
                feedback.progress_percent = float(min(100.0, 100.0 * elapsed / total_time))
                goal_handle.publish_feedback(feedback)

                time.sleep(sample_dt)

            converged = converged and settled

        arr = np.array(errors) if errors else np.array([0.0])
        result.rmse_velocity = float(np.sqrt(np.mean(arr ** 2)))
        result.iae_velocity = float(np.sum(np.abs(arr)))
        result.control_energy = float(np.sum(np.abs(controls)))
        result.converged = converged
        goal_handle.succeed()
        self.get_logger().info(
            f'Profile done: RMSE={result.rmse_velocity:.3f}, converged={converged}.')
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QuantumControllerNode()
    # Multi-threaded so the action server can run alongside the control timer.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
