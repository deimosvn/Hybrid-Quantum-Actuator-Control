"""
Classical Physics Simulator for Rover Motor Control System.

Este módulo implementa:
1. Modelo dinámico del motor DC con pérdida de tracción
2. Simulador del rover con ecuaciones diferenciales
3. Cálculo del error de control en tiempo real

Ecuaciones:
    Motor: τ = K_t * I = K_t * (V - K_e * ω) / R
    Dinámica: J * dω/dt = τ - τ_friction - τ_slip
    Error: e = ω_ref - ω_actual
"""

import numpy as np
from scipy.integrate import odeint
from dataclasses import dataclass
from typing import Tuple, Dict
import logging

logger = logging.getLogger(__name__)


@dataclass
class MotorParameters:
    """Parámetros del motor DC estándar."""
    K_t: float = 0.1          # Constante de torque (N·m/A)
    K_e: float = 0.1          # Constante de fuerza contraelectromotriz (V·s/rad)
    R: float = 2.0            # Resistencia (Ω)
    J: float = 0.01           # Inercia rotatoria (kg·m²)
    b: float = 0.05           # Amortiguamiento viscoso (N·m·s/rad)
    tau_max: float = 2.0      # Torque máximo (N·m)
    V_max: float = 12.0       # Voltaje máximo (V)
    friction_coefficient: float = 0.1  # Coeficiente de fricción


@dataclass
class RoverParameters:
    """Parámetros del rover."""
    mass: float = 5.0         # Masa total (kg)
    wheel_radius: float = 0.1 # Radio de rueda (m)
    num_wheels: float = 4.0   # Número de ruedas
    traction_loss: float = 0.0  # Factor de pérdida de tracción (0-1)


class MotorSimulator:
    """
    Simulador de motor DC con modelo dinámico realista.
    Implementa la ecuación de estado para control en bucle cerrado.
    """
    
    def __init__(self, params: MotorParameters = None):
        """
        Inicializa el simulador del motor.
        
        Args:
            params: Parámetros del motor. Si es None, usa parámetros por defecto.
        """
        self.params = params or MotorParameters()
        self.state = np.array([0.0, 0.0])  # [ω, dω/dt]
        self.dt = 0.001  # Periodo de muestreo (1 ms)
        logger.info(f"Motor inicializado con parámetros: {self.params}")
    
    def _motor_dynamics(self, state: np.ndarray, V: float, torque_external: float = 0.0) -> np.ndarray:
        """
        Ecuación diferencial del motor DC.
        
        d/dt[ω, dω/dt]^T = f(state, V, τ_ext)
        
        Args:
            state: [ω (velocidad angular), dω/dt (aceleración angular)]
            V: Voltaje aplicado (V)
            torque_external: Torque externo (carga, fricción adicional)
            
        Returns:
            Derivada del estado [dω/dt, d²ω/dt²]
        """
        omega, domega_dt = state
        
        # Clipping de voltaje
        V_clipped = np.clip(V, -self.params.V_max, self.params.V_max)
        
        # Corriente del motor: I = (V - K_e * ω) / R
        I = (V_clipped - self.params.K_e * omega) / self.params.R
        
        # Torque motor: τ_motor = K_t * I
        tau_motor = self.params.K_t * I
        
        # Torque de fricción: τ_friction = b * ω
        tau_friction = self.params.b * omega
        
        # Torque total: J * d²ω/dt² = τ_motor - τ_friction - τ_external
        d2omega_dt2 = (tau_motor - tau_friction - torque_external) / self.params.J
        
        # Clipping de aceleración para estabilidad numérica
        d2omega_dt2 = np.clip(d2omega_dt2, -50, 50)
        
        return np.array([domega_dt, d2omega_dt2])
    
    def step(self, V: float, torque_external: float = 0.0) -> Tuple[float, float]:
        """
        Integración numérica del motor (1 paso de simulación).
        Usa método de Euler explícito.
        
        Args:
            V: Voltaje aplicado (V)
            torque_external: Torque externo (N·m)
            
        Returns:
            (velocidad angular, aceleración angular)
        """
        try:
            # Integración Euler
            k1 = self._motor_dynamics(self.state, V, torque_external)
            self.state = self.state + k1 * self.dt
            
            # Saturación de velocidad angular
            self.state[0] = np.clip(self.state[0], -50, 50)  # rad/s
            
            return float(self.state[0]), float(self.state[1])
        except Exception as e:
            logger.error(f"Error en integración del motor: {e}")
            return 0.0, 0.0
    
    def reset(self):
        """Reinicia el estado del motor."""
        self.state = np.array([0.0, 0.0])
        logger.debug("Motor reseteado")
    
    def get_state(self) -> Dict[str, float]:
        """Retorna el estado actual del motor."""
        return {
            'omega': float(self.state[0]),
            'domega_dt': float(self.state[1])
        }


class RoverDynamics:
    """
    Simulador completo de dinámicas del rover.
    Modela el efecto de pérdida de tracción en las ruedas.
    """
    
    def __init__(self, motor_params: MotorParameters = None, 
                 rover_params: RoverParameters = None):
        """
        Inicializa la cinemática y dinámica del rover.
        
        Args:
            motor_params: Parámetros del motor
            rover_params: Parámetros del rover
        """
        self.motor = MotorSimulator(motor_params or MotorParameters())
        self.rover_params = rover_params or RoverParameters()
        
        # Estado del rover: [posición_lineal, velocidad_lineal]
        self.position = 0.0
        self.velocity = 0.0
        self.dt = 0.001
        
        logger.info(f"Rover inicializado con parámetros: {self.rover_params}")
    
    def _calculate_traction_loss(self, omega: float) -> float:
        """
        Calcula el torque perdido por falta de tracción.
        Modelo simplificado: pérdida proporcional a velocidad y coef. de fricción.
        
        Args:
            omega: Velocidad angular del motor (rad/s)
            
        Returns:
            Torque perdido (N·m)
        """
        # Fuerza de tracción = K_t * I, pero se pierde por deslizamiento
        v_linear = omega * self.rover_params.wheel_radius / self.rover_params.num_wheels
        
        # Torque de deslizamiento: τ_slip = m * g * μ * r * traction_loss_factor
        tau_slip = (self.rover_params.mass * 9.81 * 
                   self.rover_params.traction_loss * 
                   self.rover_params.wheel_radius)
        
        return tau_slip
    
    def step(self, V: float, reference_omega: float = 0.0) -> Dict[str, float]:
        """
        Avanza un paso de simulación del rover completo.
        
        Args:
            V: Voltaje aplicado al motor (V)
            reference_omega: Velocidad angular de referencia (rad/s)
            
        Returns:
            Diccionario con estado del sistema completo
        """
        try:
            # Calcular pérdida de tracción
            tau_slip = self._calculate_traction_loss(self.motor.state[0])
            
            # Paso del motor
            omega, domega_dt = self.motor.step(V, torque_external=tau_slip)
            
            # Actualizar posición y velocidad lineales
            v_linear = (omega * self.rover_params.wheel_radius / 
                       self.rover_params.num_wheels)
            self.velocity = v_linear
            self.position += self.velocity * self.dt
            
            # Calcular error de control
            error = reference_omega - omega
            
            return {
                'position': self.position,
                'velocity': self.velocity,
                'omega': omega,
                'domega_dt': domega_dt,
                'error': error,
                'tau_slip': tau_slip,
                'reference_omega': reference_omega
            }
        except Exception as e:
            logger.error(f"Error en dinámica del rover: {e}")
            return {
                'position': self.position,
                'velocity': 0.0,
                'omega': 0.0,
                'domega_dt': 0.0,
                'error': 0.0,
                'tau_slip': 0.0,
                'reference_omega': reference_omega
            }
    
    def reset(self):
        """Reinicia el estado del rover."""
        self.motor.reset()
        self.position = 0.0
        self.velocity = 0.0
        logger.debug("Rover reseteado")
    
    def get_state(self) -> Dict[str, float]:
        """Retorna el estado completo del rover."""
        return {
            'position': self.position,
            'velocity': self.velocity,
            'motor': self.motor.get_state()
        }
