# Copyright 2026 Diego Eduardo Martinez Cruz
# SPDX-License-Identifier: MIT
"""Core module for Quantum-Classical Hybrid Rover Control System.

Contains quantum optimization and classical physics simulation.
"""

from .classical_physics import MotorSimulator, RoverDynamics, MotorParameters, RoverParameters
from .quantum_optimizer import QuantumOptimizer, QAOAController, FallbackController

__all__ = [
    'QuantumOptimizer',
    'QAOAController',
    'FallbackController',
    'MotorSimulator',
    'RoverDynamics',
    'MotorParameters',
    'RoverParameters'
]
