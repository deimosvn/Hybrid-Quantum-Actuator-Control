"""
Optimizador Cuántico para Control del Rover.

Implementa QAOA (Quantum Approximate Optimization Algorithm) para minimizar
la función de costo: J = x^T Q x + u^T R u

donde:
    x: vector de estado [error de posición, velocidad del motor]
    u: signal de control (voltaje PWM)
    Q, R: matrices de peso (sintonización del control)

El circuito cuántico codifica el problema de optimización y usa
variational optimization para encontrar parámetros óptimos.
"""

import numpy as np
from typing import Tuple, Dict, Optional, List
import logging
from abc import ABC, abstractmethod

try:
    from qiskit import QuantumCircuit, QuantumRegister, ClassicalRegister
    from qiskit_aer import AerSimulator
    from qiskit.quantum_info import Operator
    from qiskit.primitives import Estimator
    from qiskit.algorithms.optimizers import COBYLA, SPSA
    from scipy.optimize import minimize
    QISKIT_AVAILABLE = True
except ImportError as e:
    QISKIT_AVAILABLE = False
    logger = logging.getLogger(__name__)
    logger.warning(f"Qiskit no disponible: {e}. Usando fallback clásico.")

logger = logging.getLogger(__name__)


class QuantumOptimizer(ABC):
    """
    Interfaz base para optimizadores cuánticos.
    Permite múltiples implementaciones (QAOA, VQE, etc.)
    """
    
    @abstractmethod
    def optimize(self, error_state: np.ndarray) -> np.ndarray:
        """
        Optimiza el control dado el estado de error.
        
        Args:
            error_state: Vector de estado [error_posición, velocidad]
            
        Returns:
            Vector de control óptimo
        """
        pass
    
    @abstractmethod
    def cost_function(self, x: np.ndarray, Q: np.ndarray, R: np.ndarray) -> float:
        """Calcula el costo del control."""
        pass


class QAOACircuit:
    """
    Circuito cuántico parametrizado para QAOA.
    Implementa la estructura: Hadamard -> Cost Hamiltonian -> Mixer Hamiltonian
    """
    
    def __init__(self, num_qubits: int = 2, num_layers: int = 1):
        """
        Inicializa el circuito QAOA.
        
        Args:
            num_qubits: Número de qubits (típicamente 2-4 para este problema)
            num_layers: Número de capas QAOA (p-parameter)
        """
        self.num_qubits = num_qubits
        self.num_layers = num_layers
        self.num_parameters = 2 * num_layers  # γ (cost) y β (mixer)
        
        logger.info(f"QAOA Circuit: {num_qubits} qubits, {num_layers} capas")
    
    def build_circuit(self, params: np.ndarray) -> 'QuantumCircuit':
        """
        Construye el circuito QAOA con parámetros dados.
        
        Estructura:
        1. Estado inicial: superposición uniforme (Hadamard en todos)
        2. Capas QAOA: alternancia de Cost y Mixer Hamiltonian
        3. Medición en base Z
        
        Args:
            params: Array [γ_0, β_0, γ_1, β_1, ...] parámetros variacionales
            
        Returns:
            Circuito cuántico construido
        """
        if not QISKIT_AVAILABLE:
            raise RuntimeError("Qiskit no está disponible")
        
        qc = QuantumCircuit(self.num_qubits, self.num_qubits, name="QAOA")
        
        # 1. Inicialización: superposición uniforme
        for i in range(self.num_qubits):
            qc.h(i)
        
        # 2. Capas QAOA
        for layer in range(self.num_layers):
            gamma = params[2 * layer]
            beta = params[2 * layer + 1]
            
            # Cost Hamiltonian: ZZ interactions
            # Implementa el acoplamiento entre qubits
            for i in range(self.num_qubits - 1):
                qc.rzz(2 * gamma * 0.5, i, i + 1)  # e^(-i*γ*Z_i*Z_{i+1})
            
            # Diagonal cost terms
            for i in range(self.num_qubits):
                qc.rz(2 * gamma * 0.5, i)
            
            # Mixer Hamiltonian: X rotations
            for i in range(self.num_qubits):
                qc.rx(2 * beta, i)
        
        # 3. Medición
        qc.measure(range(self.num_qubits), range(self.num_qubits))
        
        return qc
    
    def get_expectation_circuit(self, params: np.ndarray) -> 'QuantumCircuit':
        """
        Construye circuito para evaluar valor esperado (sin medición final).
        Usado para cálculo de cost function.
        
        Args:
            params: Parámetros variacionales
            
        Returns:
            Circuito para evaluación de expectativa
        """
        if not QISKIT_AVAILABLE:
            raise RuntimeError("Qiskit no está disponible")
        
        qc = QuantumCircuit(self.num_qubits, name="QAOA_Expectation")
        
        # Hadamard inicial
        for i in range(self.num_qubits):
            qc.h(i)
        
        # Capas QAOA
        for layer in range(self.num_layers):
            gamma = params[2 * layer]
            beta = params[2 * layer + 1]
            
            for i in range(self.num_qubits - 1):
                qc.rzz(2 * gamma * 0.5, i, i + 1)
            
            for i in range(self.num_qubits):
                qc.rz(2 * gamma * 0.5, i)
            
            for i in range(self.num_qubits):
                qc.rx(2 * beta, i)
        
        return qc


class QAOAController(QuantumOptimizer):
    """
    Controlador de rover usando QAOA.
    Resuelve el problema de optimización: min(x^T Q x + u^T R u)
    """
    
    def __init__(self, 
                 num_qubits: int = 2,
                 num_layers: int = 1,
                 use_simulator: bool = True):
        """
        Inicializa el controlador QAOA.
        
        Args:
            num_qubits: Número de qubits (2-4 recomendado)
            num_layers: Capas QAOA (1-2 típico)
            use_simulator: Si True, usa AerSimulator; si False, usa versión clásica
        """
        if not QISKIT_AVAILABLE and use_simulator:
            logger.warning("Qiskit indisponible, usando fallback clásico")
            use_simulator = False
        
        self.num_qubits = num_qubits
        self.num_layers = num_layers
        self.use_simulator = use_simulator
        
        # Parámetros de sintonización LQR
        self.Q = np.diag([10.0, 1.0])  # Penalización por error de estado
        self.R = np.array([[0.5]])     # Penalización por esfuerzo de control
        
        # Circuito QAOA
        self.qaoa_circuit = QAOACircuit(num_qubits, num_layers)
        
        # Simulador cuántico
        if use_simulator and QISKIT_AVAILABLE:
            self.simulator = AerSimulator(method='statevector')
        else:
            self.simulator = None
        
        # Almacenamiento de histórico
        self.optimization_history: List[Dict] = []
        self.params_optimal = np.random.randn(2 * num_layers) * 0.1
        
        logger.info(f"QAOAController inicializado: {num_qubits} qubits, "
                   f"{num_layers} capas, simulador: {use_simulator}")
    
    def cost_function(self, x: np.ndarray, Q: np.ndarray, R: np.ndarray) -> float:
        """
        Calcula el costo del control.
        J = ||e||_Q² + ||u||_R²
        
        Args:
            x: Vector de estado [error, velocidad]
            Q: Matriz de peso de estado
            R: Matriz de peso de control
            
        Returns:
            Costo total (escalar)
        """
        state_cost = x[:2].T @ Q @ x[:2]  # Costo de estado
        control_cost = x[2:]**2  # Costo de control (u² con R=1)
        return float(state_cost + control_cost)
    
    def _encode_state_to_qubits(self, error_state: np.ndarray) -> np.ndarray:
        """
        Codifica el estado clásico en parámetros iniciales del circuito.
        
        Mapeo: estado normalizado -> rotaciones iniciales de qubits
        
        Args:
            error_state: [error_posición, velocidad_motor]
            
        Returns:
            Parámetros iniciales para circuito
        """
        # Normalización
        error_norm = np.linalg.norm(error_state) + 1e-8
        error_normalized = error_state / error_norm
        
        # Mapeo a ángulos [0, π]
        angles = np.arccos(np.clip(error_normalized, -1, 1))
        
        return angles
    
    def _decode_qubits_to_control(self, bitstring: str) -> float:
        """
        Decodifica el resultado de medición a señal de control.
        
        Args:
            bitstring: Resultado de medición (e.g., "01")
            
        Returns:
            Voltaje de control normalizado [-1, 1]
        """
        # Conversión binaria a decimal
        decimal = int(bitstring, 2)
        
        # Normalización al rango [-1, 1]
        max_val = 2**self.num_qubits - 1
        control = 2.0 * (decimal / max_val) - 1.0
        
        return float(control)
    
    def optimize(self, error_state: np.ndarray, learning_rate: float = 0.01) -> float:
        """
        Ejecuta optimización QAOA sobre el estado de error.
        Encuentra parámetros variacionales que minimizan la función de costo.
        
        Args:
            error_state: Vector [error_posición, velocidad] del sistema
            learning_rate: Tasa de aprendizaje para SGD
            
        Returns:
            Control óptimo normalizado [-1, 1]
        """
        try:
            if self.use_simulator and QISKIT_AVAILABLE:
                return self._optimize_quantum(error_state)
            else:
                return self._optimize_classical(error_state)
        except Exception as e:
            logger.error(f"Error en optimización: {e}")
            return 0.0
    
    def _optimize_quantum(self, error_state: np.ndarray) -> float:
        """
        Optimización cuántica usando QAOA con Qiskit.
        """
        from qiskit.primitives import Estimator
        
        def objective(params):
            """Función objetivo a minimizar."""
            try:
                # Construir circuito
                qc = self.qaoa_circuit.build_circuit(params)
                
                # Estimar expectativa de energía (simulación)
                job = self.simulator.run(qc, shots=1000)
                result = job.result()
                counts = result.get_counts()
                
                # Calcular valor esperado
                expectation = 0.0
                for bitstring, count in counts.items():
                    control = self._decode_qubits_to_control(bitstring)
                    prob = count / 1000
                    
                    # Costo: J = e^T Q e + u^2
                    cost = (error_state[0]**2 * self.Q[0, 0] + 
                           error_state[1]**2 * self.Q[1, 1] + 
                           control**2 * self.R[0, 0])
                    expectation += prob * cost
                
                return expectation
            except Exception as e:
                logger.warning(f"Error en evaluación objetiva: {e}")
                return 1e6
        
        # Optimización clásica de parámetros
        optimizer = COBYLA(maxiter=50)
        result = minimize(objective, self.params_optimal, 
                         method='COBYLA', options={'maxiter': 50})
        
        self.params_optimal = result.x
        
        # Evaluar con parámetros óptimos
        qc = self.qaoa_circuit.build_circuit(self.params_optimal)
        job = self.simulator.run(qc, shots=1000)
        counts = job.result().get_counts()
        
        # Seleccionar bitstring más probable
        best_bitstring = max(counts, key=counts.get)
        control = self._decode_qubits_to_control(best_bitstring)
        
        logger.debug(f"QAOA optimization: {best_bitstring} -> control: {control:.4f}")
        return control
    
    def _optimize_classical(self, error_state: np.ndarray) -> float:
        """
        Optimización clásica aproximada (fallback cuando Qiskit no disponible).
        Usa gradient descent en parámetros binarios.
        """
        # Aproximación clásica: LQR básico
        K_gain = 0.1  # Ganancia proporcional
        control = -K_gain * error_state[0]  # PD simple
        
        # Saturación
        control = np.clip(control, -1.0, 1.0)
        
        logger.debug(f"Classical optimization: control = {control:.4f}")
        return float(control)
    
    def update_cost_matrices(self, Q: Optional[np.ndarray] = None, 
                            R: Optional[np.ndarray] = None):
        """
        Actualiza matrices de costo Q y R para sintonización.
        
        Args:
            Q: Nueva matriz de pesos de estado (2x2)
            R: Nueva matriz de pesos de control (1x1)
        """
        if Q is not None:
            self.Q = Q
        if R is not None:
            self.R = R
        logger.info(f"Matrices de costo actualizadas: Q shape={self.Q.shape}, R shape={self.R.shape}")


class FallbackController(QuantumOptimizer):
    """
    Controlador clásico PID basado en realimentación como fallback.
    Se usa cuando Qiskit no está disponible.
    """
    
    def __init__(self):
        """Inicializa parámetros PID."""
        self.kp = 0.5
        self.ki = 0.1
        self.kd = 0.05
        self.integrator = 0.0
        self.last_error = 0.0
        self.Q = np.diag([10.0, 1.0])
        self.R = np.array([[0.5]])
    
    def cost_function(self, x: np.ndarray, Q: np.ndarray, R: np.ndarray) -> float:
        """Calcula costo (ver docstring padre)."""
        return float(x[:2].T @ Q @ x[:2] + x[2:]**2)
    
    def optimize(self, error_state: np.ndarray) -> float:
        """
        Control PID clásico.
        
        Args:
            error_state: [error_posición, velocidad]
            
        Returns:
            Señal de control
        """
        error = error_state[0]
        
        # Proporcional
        p_term = self.kp * error
        
        # Integral
        self.integrator += error * 0.001
        i_term = self.ki * self.integrator
        
        # Derivativo
        d_term = self.kd * (error - self.last_error) / 0.001
        self.last_error = error
        
        # Control total
        control = p_term + i_term + d_term
        
        # Saturación
        return float(np.clip(control, -1.0, 1.0))
