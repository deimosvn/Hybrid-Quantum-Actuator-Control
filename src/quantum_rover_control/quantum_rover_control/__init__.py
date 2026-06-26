# Copyright 2026 Diego Eduardo Martinez Cruz
# SPDX-License-Identifier: MIT
"""Hybrid quantum-classical rover actuator control for ROS 2.

This package bundles:
  * ``core``  -- ROS-agnostic plant physics and the QAOA / PID optimizers.
  * ``utils`` -- telemetry dataclasses and offline logging / metrics.
  * ``nodes`` -- rclpy nodes that wrap the algorithms in a ROS 2 graph.

The ``core`` and ``utils`` sub-packages carry no ROS dependency, so the control
algorithms can be unit-tested and run as a standalone simulation
(``quantum_rover_control.sim_demo``) without a ROS 2 installation.
"""

__version__ = "1.0.0"
