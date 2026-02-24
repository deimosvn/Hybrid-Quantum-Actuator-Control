"""
Core module for Quantum-Classical Hybrid Rover Control System.
Contains quantum optimization and classical physics simulation.
"""

from .quantum_optimizer import QuantumOptimizer, QAOAController, FallbackController
from .classical_physics import MotorSimulator, RoverDynamics, MotorParameters, RoverParameters

__all__ = [
    'QuantumOptimizer',
    'QAOAController',
    'FallbackController',
    'MotorSimulator',
    'RoverDynamics',
    'MotorParameters',
    'RoverParameters'
]
