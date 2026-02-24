"""
Sistema de Control Híbrido Cuántico-Clásico para Rover.

Punto de entrada principal que integra:
1. Simulador clásico de dinámicas del control
2. Optimizador cuántico QAOA
3. Bucle de realimentación en tiempo real
4. Logging y análisis comparativo

Autor: Quantum Mechatronics Lab
Fecha: 2026
"""

import numpy as np
import logging
from typing import Tuple, Dict, Optional
import time
from pathlib import Path

# Importaciones locales
from core import (
    QuantumOptimizer, QAOAController, FallbackController,
    RoverDynamics, MotorParameters, RoverParameters
)
from utils import DataLogger, TelemetryRecord

# Configurar logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class HybridRoverController:
    """
    Controlador híbrido que coordina simulación clásica y optimización cuántica.
    Implementa el bucle de realimentación en tiempo real.
    """
    
    def __init__(self, 
                 use_quantum: bool = True,
                 quantum_num_qubits: int = 2,
                 quantum_num_layers: int = 1):
        """
        Inicializa el sistema de control híbrido.
        
        Args:
            use_quantum: Si True, usa QAOA; si False usa controlador clásico
            quantum_num_qubits: Número de qubits para QAOA
            quantum_num_layers: Número de capas QAOA
        """
        # Parámetros del sistema
        self.motor_params = MotorParameters(
            K_t=0.15,
            K_e=0.15,
            R=1.5,
            J=0.01,
            b=0.03,
            friction_coefficient=0.15
        )
        
        self.rover_params = RoverParameters(
            mass=5.0,
            wheel_radius=0.1,
            traction_loss=0.2  # Pérdida de tracción del 20%
        )
        
        # Dinámicas del rover
        self.rover = RoverDynamics(self.motor_params, self.rover_params)
        
        # Optimizador
        if use_quantum:
            try:
                self.optimizer: QuantumOptimizer = QAOAController(
                    num_qubits=quantum_num_qubits,
                    num_layers=quantum_num_layers,
                    use_simulator=True
                )
                self.control_mode = 'quantum'
                logger.info("Control CUÁNTICO (QAOA) activado")
            except Exception as e:
                logger.warning(f"QAOA falló: {e}, usando fallback clásico")
                self.optimizer = FallbackController()
                self.control_mode = 'classical'
        else:
            self.optimizer = FallbackController()
            self.control_mode = 'classical'
            logger.info("Control CLÁSICO (PID) activado")
        
        # Logger de telemetría
        self.logger = DataLogger(output_dir="./logs")
        
        # Referencias de control
        self.reference_omega = 5.0  # rad/s
        self.reference_position = 0.0
        
        # Historial
        self.iteration = 0
        self.start_time = time.time()
    
    def _calculate_error_state(self, rover_state: Dict) -> np.ndarray:
        """
        Calcula el vector de estado de error para el optimizador.
        
        Args:
            rover_state: Diccionario con estado del rover
            
        Returns:
            Vector [error_posición, error_velocidad]
        """
        position_error = self.reference_position - rover_state['position']
        velocity_error = self.reference_omega - rover_state['omega']
        
        return np.array([position_error, velocity_error])
    
    def _pwm_from_control(self, control: float, pwm_max: float = 12.0) -> float:
        """
        Convierte señal de control normalizada [-1, 1] a PWM en Voltios.
        
        Args:
            control: Señal de control normalizada [-1, 1]
            pwm_max: Voltaje máximo del PWM (V)
            
        Returns:
            Voltaje a aplicar al motor (V)
        """
        voltage = control * pwm_max
        return float(np.clip(voltage, -pwm_max, pwm_max))
    
    def control_step(self) -> Tuple[Dict, TelemetryRecord]:
        """
        Ejecuta un paso de simulación y control (1 kHz).
        
        Algoritmo:
        1. Leer estado actual del rover
        2. Calcular error
        3. Ejecutar optimizador (cuántico o clásico)
        4. Aplicar control
        5. Integrar dinámica
        6. Registrar telemetría
        
        Returns:
            (estado_rover, registro_telemetría)
        """
        step_start_time = time.time()
        
        try:
            # 1. Leer estado
            rover_state = self.rover.get_state()
            motor_state = rover_state['motor']
            
            # 2. Calcular error
            error_state = self._calculate_error_state(rover_state)
            
            # 3. Optimizar control
            opt_start = time.time()
            control_normalized = self.optimizer.optimize(error_state)
            opt_time_ms = (time.time() - opt_start) * 1000
            
            # 4. Convertir a voltaje
            voltage = self._pwm_from_control(control_normalized)
            
            # 5. Simular dinámica
            state = self.rover.step(voltage, self.reference_omega)
            
            # 6. Calcular costo
            cost = self.optimizer.cost_function(
                np.array([error_state[0], error_state[1], control_normalized]),
                self.optimizer.Q,
                self.optimizer.R
            )
            
            # 7. Crear registro de telemetría
            elapsed_time = time.time() - self.start_time
            
            record = TelemetryRecord(
                timestamp=elapsed_time,
                iteration=self.iteration,
                position=state['position'],
                velocity=state['velocity'],
                motor_omega=state['omega'],
                motor_domega_dt=state['domega_dt'],
                control_input=voltage,
                control_source=self.control_mode,
                position_error=error_state[0],
                velocity_error=error_state[1],
                tau_slip=state['tau_slip'],
                cost_function=cost,
                optimization_time_ms=opt_time_ms,
                qiskit_success=(self.control_mode == 'quantum')
            )
            
            self.logger.log(record)
            self.iteration += 1
            
            return state, record
        
        except Exception as e:
            logger.error(f"Error en paso de control: {e}")
            return None, None
    
    def run_simulation(self, duration: float = 10.0, 
                      frequency: float = 1000.0) -> bool:
        """
        Ejecuta simulación completa del sistema.
        
        Args:
            duration: Duración de la simulación (segundos)
            frequency: Frecuencia de simulación (Hz)
            
        Returns:
            True si completó exitosamente
        """
        logger.info(f"Iniciando simulación: {duration}s @ {frequency}Hz")
        logger.info(f"Modo de control: {self.control_mode}")
        
        num_steps = int(duration * frequency)
        dt = 1.0 / frequency
        
        print("\n" + "="*70)
        print("SIMULACIÓN INICIADA")
        print(f"Duración: {duration}s | Frecuencia: {frequency}Hz")
        print(f"Pasos totales: {num_steps}")
        print("="*70 + "\n")
        
        try:
            for step in range(num_steps):
                # Ejecutar paso de control
                state, record = self.control_step()
                
                # Actualizar referencia (rampa lineal)
                if step < num_steps // 3:
                    self.reference_omega = 5.0
                elif step < 2 * num_steps // 3:
                    self.reference_omega = 8.0
                else:
                    self.reference_omega = 3.0
                
                # Mostrar progreso cada 1000 pasos (1 segundo)
                if (step + 1) % int(frequency) == 0:
                    elapsed = step / frequency
                    progress = 100 * step / num_steps
                    print(f"[{progress:5.1f}%] t={elapsed:6.2f}s | "
                          f"ω={state['omega']:6.2f} rad/s | "
                          f"e={state['error']:6.3f} | "
                          f"u={record.control_input:6.2f}V")
                
                # Dormir para mantener tiempo real (opcional)
                # time.sleep(dt)
            
            logger.info("Simulación completada exitosamente")
            return True
        
        except KeyboardInterrupt:
            logger.warning("Simulación interrumpida por usuario")
            return False
        except Exception as e:
            logger.error(f"Error crítico en simulación: {e}")
            return False
    
    def analyze_results(self):
        """Analiza resultados y genera reportes."""
        logger.info("Analizando resultados...")
        
        # Imprimir resumen
        self.logger.print_summary()
        
        # Guardar datos
        csv_file = self.logger.save_csv("hybrid_control")
        json_file = self.logger.save_json("hybrid_control")
        
        # Generar gráficos
        try:
            plot_file = self.logger.plot_comparison(show=False)
            if plot_file:
                logger.info(f"Gráficos guardados en {plot_file}")
        except ImportError:
            logger.warning("matplotlib no disponible para gráficos")
        
        # Métricas
        metrics = self.logger.compute_metrics()
        
        return {
            'csv_file': csv_file,
            'json_file': json_file,
            'metrics': metrics
        }


def main():
    """
    Función principal - Ejecuta demostración completa del sistema.
    """
    
    # Crear controlador híbrido
    controller = HybridRoverController(
        use_quantum=True,
        quantum_num_qubits=2,
        quantum_num_layers=1
    )
    
    # Ejecutar simulación
    success = controller.run_simulation(
        duration=15.0,  # 15 segundos
        frequency=100.0  # 100 Hz (más rápido para demostración)
    )
    
    if success:
        # Analizar y reportar
        results = controller.analyze_results()
        
        print("\n" + "="*70)
        print("SIMULACIÓN COMPLETADA")
        print("="*70)
        print(f"Archivos de salida:")
        print(f"  CSV: {results['csv_file']}")
        print(f"  JSON: {results['json_file']}")
        print("="*70 + "\n")
    else:
        print("Simulación no completada exitosamente")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
