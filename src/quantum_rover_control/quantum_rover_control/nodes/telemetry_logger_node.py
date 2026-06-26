#!/usr/bin/env python3
# Copyright 2026 Diego Eduardo Martinez Cruz
# SPDX-License-Identifier: MIT
"""Telemetry logger node.

Subscribes to the controller's ``RoverTelemetry`` stream and persists it through
the existing :class:`quantum_rover_control.utils.DataLogger`, producing CSV/JSON
exports, performance metrics and (optionally) comparison plots. Designed to run
for the duration of a benchmark and flush on shutdown.

Subscribes
----------
``~/telemetry`` : :class:`quantum_rover_interfaces.msg.RoverTelemetry`
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from quantum_rover_control.utils import DataLogger, TelemetryRecord
from quantum_rover_interfaces.msg import RoverTelemetry


class TelemetryLoggerNode(Node):
    """Persists telemetry and computes quantum-vs-classical metrics."""

    def __init__(self) -> None:
        super().__init__('telemetry_logger')

        self.declare_parameter('output_dir', './logs')
        self.declare_parameter('file_prefix', 'hybrid_control')
        self.declare_parameter('save_plots', True)
        self.declare_parameter('flush_on_shutdown', True)

        output_dir = self.get_parameter('output_dir').value
        self._prefix = self.get_parameter('file_prefix').value
        self._logger = DataLogger(output_dir=output_dir)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(RoverTelemetry, 'telemetry', self._on_telemetry, qos)

        self.get_logger().info(f'Telemetry logger writing to {output_dir!r}.')

    def _on_telemetry(self, msg: RoverTelemetry) -> None:
        record = TelemetryRecord(
            timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            iteration=int(msg.iteration),
            position=msg.position,
            velocity=msg.velocity,
            motor_omega=msg.motor.omega,
            motor_domega_dt=msg.motor.domega_dt,
            control_input=msg.control_input,
            control_source=msg.control_source,
            position_error=msg.position_error,
            velocity_error=msg.velocity_error,
            tau_slip=msg.tau_slip,
            cost_function=msg.cost_function,
            optimization_time_ms=msg.optimization_time_ms,
            qiskit_success=msg.quantum_backend_ok,
        )
        self._logger.log(record)

    def flush(self) -> None:
        """Write CSV/JSON, print the summary and (optionally) plots."""
        if not self._logger.records:
            self.get_logger().warn('No telemetry received; nothing to flush.')
            return
        self._logger.print_summary()
        csv_path = self._logger.save_csv(self._prefix)
        json_path = self._logger.save_json(self._prefix)
        self.get_logger().info(f'Saved {csv_path} and {json_path}.')
        if self.get_parameter('save_plots').value:
            try:
                plot = self._logger.plot_comparison(show=False)
                if plot:
                    self.get_logger().info(f'Saved comparison plot {plot}.')
            except Exception as exc:  # noqa: BLE001 - plotting is best-effort
                self.get_logger().warn(f'Plot generation skipped: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.get_parameter('flush_on_shutdown').value:
            node.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
