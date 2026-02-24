"""
Core module for Quantum-Classical Hybrid Rover Control System.
Contains quantum optimization and classical physics simulation.
"""

from .quantum_optimizer import QuantumOptimizer, QAOAController
from .classical_physics import MotorSimulator, RoverDynamics

__all__ = [
    'QuantumOptimizer',
    'QAOAController',
    'MotorSimulator',
    'RoverDynamics'
]
