# Copyright 2026 Diego Eduardo Martinez Cruz
# SPDX-License-Identifier: MIT
"""Unit tests for the ROS-agnostic control algorithms and plant model."""

import numpy as np
import pytest

from quantum_rover_control.core import (
    FallbackController,
    MotorParameters,
    QAOAController,
    RoverDynamics,
    RoverParameters,
)


def test_fallback_controller_output_saturates():
    """The PID fallback must always return a control within [-1, 1]."""
    controller = FallbackController()
    for error in (-100.0, -1.0, 0.0, 1.0, 100.0):
        action = controller.optimize(np.array([error, 0.0]))
        assert -1.0 <= action <= 1.0


def test_fallback_controller_reacts_to_positive_error():
    """A positive position error should drive a positive control effort."""
    controller = FallbackController()
    action = controller.optimize(np.array([1.0, 0.0]))
    assert action > 0.0


def test_cost_function_is_non_negative():
    """LQR cost J = x^T Q x + u^2 is positive semi-definite."""
    controller = FallbackController()
    cost = controller.cost_function(
        np.array([0.5, -0.3, 0.2]), controller.Q, controller.R)
    assert cost >= 0.0


def test_qaoa_controller_falls_back_without_simulator():
    """With the simulator disabled, QAOAController uses the classical path."""
    controller = QAOAController(num_qubits=2, num_layers=1, use_simulator=False)
    action = controller.optimize(np.array([0.5, 0.1]))
    assert isinstance(action, float)
    assert -1.0 <= action <= 1.0


def test_rover_dynamics_accelerates_under_voltage():
    """Applying a positive voltage should spin the motor up from rest."""
    rover = RoverDynamics(MotorParameters(), RoverParameters())
    omega = 0.0
    for _ in range(200):
        state = rover.step(6.0, reference_omega=5.0)
        omega = state['omega']
    assert omega > 0.0
    assert set(state).issuperset(
        {'position', 'velocity', 'omega', 'error', 'tau_slip'})


def test_traction_loss_increases_slip_torque():
    """Higher traction loss yields a larger slip torque magnitude."""
    low = RoverDynamics(MotorParameters(), RoverParameters(traction_loss=0.1))
    high = RoverDynamics(MotorParameters(), RoverParameters(traction_loss=0.5))
    low_state = low.step(6.0)
    high_state = high.step(6.0)
    assert high_state['tau_slip'] > low_state['tau_slip']


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
