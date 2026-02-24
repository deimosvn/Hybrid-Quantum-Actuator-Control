"""
Sistema de Logging de Telemetría para Comparación de Controles.

Proporciona:
1. Grabación de datos en tiempo real
2. Exportación a CSV para análisis
3. Métricas de desempeño (RMSE, IAE, etc.)
4. Comparación cuántico vs clásico
"""

import json
import csv
from dataclasses import dataclass, asdict, field
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Optional
import numpy as np
import logging

logger = logging.getLogger(__name__)


@dataclass
class TelemetryRecord:
    """
    Un registro individual de telemetría del sistema.
    Captura estado completo en un instante de tiempo.
    """
    timestamp: float
    iteration: int
    
    # Estado del rover
    position: float
    velocity: float
    motor_omega: float
    motor_domega_dt: float
    
    # Control
    control_input: float  # Voltaje PWM
    control_source: str  # 'quantum' o 'classical'
    
    # Error
    position_error: float
    velocity_error: float
    
    # Energía y dinámicas
    tau_slip: float  # Torque perdido por tracción
    cost_function: float  # Costo J = x^T Q x + u^T R u
    
    # Señales de diagnóstico
    optimization_time_ms: float = 0.0
    qiskit_success: bool = True


class DataLogger:
    """
    Logger de telemetría para análisis comparativo de controles.
    Almacena datos de simulación y genera métricas de desempeño.
    """
    
    def __init__(self, output_dir: str = "./logs"):
        """
        Inicializa el logger.
        
        Args:
            output_dir: Directorio donde se almacenan logs (créase si no existe)
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.records: List[TelemetryRecord] = []
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.quantum_records: List[TelemetryRecord] = []
        self.classical_records: List[TelemetryRecord] = []
        
        logger.info(f"DataLogger inicializado en {self.output_dir}")
    
    def log(self, record: TelemetryRecord):
        """
        Registra un evento de telemetría.
        
        Args:
            record: Registro de telemetría a almacenar
        """
        self.records.append(record)
        
        # Separar por fuente de control
        if record.control_source == 'quantum':
            self.quantum_records.append(record)
        else:
            self.classical_records.append(record)
    
    def save_csv(self, prefix: str = "telemetry"):
        """
        Exporta datos a CSV para análisis en MATLAB/Python.
        
        Args:
            prefix: Prefijo del archivo de salida
            
        Returns:
            Path al archivo creado
        """
        if not self.records:
            logger.warning("No hay registros para guardar")
            return None
        
        filename = self.output_dir / f"{prefix}_{self.session_id}.csv"
        
        try:
            with open(filename, 'w', newline='') as f:
                fieldnames = [
                    'timestamp', 'iteration', 'position', 'velocity',
                    'motor_omega', 'motor_domega_dt', 'control_input',
                    'control_source', 'position_error', 'velocity_error',
                    'tau_slip', 'cost_function', 'optimization_time_ms',
                    'qiskit_success'
                ]
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                
                for record in self.records:
                    writer.writerow(asdict(record))
            
            logger.info(f"Datos exportados a {filename}")
            return filename
        except Exception as e:
            logger.error(f"Error guardando CSV: {e}")
            return None
    
    def save_json(self, prefix: str = "telemetry"):
        """
        Exporta datos a JSON (formato legible con estructura anidada).
        
        Args:
            prefix: Prefijo del archivo
            
        Returns:
            Path al archivo creado
        """
        if not self.records:
            logger.warning("No hay registros para guardar")
            return None
        
        filename = self.output_dir / f"{prefix}_{self.session_id}.json"
        
        try:
            data = {
                'session_id': self.session_id,
                'total_samples': len(self.records),
                'quantum_samples': len(self.quantum_records),
                'classical_samples': len(self.classical_records),
                'records': [asdict(r) for r in self.records]
            }
            
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            
            logger.info(f"Datos exportados a {filename}")
            return filename
        except Exception as e:
            logger.error(f"Error guardando JSON: {e}")
            return None
    
    def compute_metrics(self) -> Dict[str, Dict[str, float]]:
        """
        Calcula métricas de desempeño para cada controlador.
        
        Métricas:
        - RMSE (Root Mean Square Error): medida de error general
        - IAE (Integral Absolute Error): integral del error absoluto
        - ISE (Integral Square Error): integral del error cuadrático
        - Control Energy: suma de esfuerzo de control
        - Mean Cost: promedio de función de costo
        
        Returns:
            Diccionario con métricas de cada controlador
        """
        metrics = {
            'quantum': self._compute_controller_metrics(self.quantum_records),
            'classical': self._compute_controller_metrics(self.classical_records)
        }
        
        # Comparativa
        if self.quantum_records and self.classical_records:
            quantum_rmse = metrics['quantum'].get('rmse_position', 0)
            classical_rmse = metrics['classical'].get('rmse_position', 0)
            
            improvement = ((classical_rmse - quantum_rmse) / classical_rmse * 100
                          if classical_rmse > 0 else 0)
            
            metrics['comparison'] = {
                'quantum_better': quantum_rmse < classical_rmse,
                'rmse_improvement_percent': improvement
            }
        
        return metrics
    
    def _compute_controller_metrics(self, records: List[TelemetryRecord]) -> Dict[str, float]:
        """
        Calcula métricas para un conjunto de registros.
        
        Args:
            records: Lista de registros a analizar
            
        Returns:
            Diccionario de métricas
        """
        if not records:
            return {}
        
        position_errors = np.array([r.position_error for r in records])
        velocities = np.array([r.velocity for r in records])
        controls = np.array([r.control_input for r in records])
        costs = np.array([r.cost_function for r in records])
        
        metrics = {
            'rmse_position': float(np.sqrt(np.mean(position_errors**2))),
            'iae_position': float(np.sum(np.abs(position_errors))),
            'ise_position': float(np.sum(position_errors**2)),
            'rmse_velocity': float(np.sqrt(np.mean(velocities**2))),
            'control_energy': float(np.sum(np.abs(controls))),
            'mean_cost': float(np.mean(costs)),
            'max_cost': float(np.max(costs)),
            'optimization_time_mean_ms': float(
                np.mean([r.optimization_time_ms for r in records])
            )
        }
        
        return metrics
    
    def print_summary(self):
        """Imprime resumen de métricas en consola."""
        metrics = self.compute_metrics()
        
        print("\n" + "="*70)
        print("RESUMEN DE TELEMETRÍA - CONTROL HÍBRIDO CUÁNTICO-CLÁSICO")
        print("="*70)
        print(f"Session ID: {self.session_id}")
        print(f"Total de muestras: {len(self.records)}")
        print(f"  - Cuánticas: {len(self.quantum_records)}")
        print(f"  - Clásicas: {len(self.classical_records)}")
        
        # Métricas cuánticas
        if 'quantum' in metrics and metrics['quantum']:
            print("\n--- CONTROLADOR CUÁNTICO (QAOA) ---")
            for key, value in metrics['quantum'].items():
                print(f"  {key:30s}: {value:12.6f}")
        
        # Métricas clásicas
        if 'classical' in metrics and metrics['classical']:
            print("\n--- CONTROLADOR CLÁSICO (PID) ---")
            for key, value in metrics['classical'].items():
                print(f"  {key:30s}: {value:12.6f}")
        
        # Comparativa
        if 'comparison' in metrics:
            comparison = metrics['comparison']
            print("\n--- COMPARATIVA ---")
            if comparison['quantum_better']:
                print(f"  ✓ Controlador CUÁNTICO es superior")
            else:
                print(f"  ✗ Controlador CLÁSICO es superior")
            print(f"  Mejora en RMSE: {comparison['rmse_improvement_percent']:.2f}%")
        
        print("="*70 + "\n")
    
    def plot_comparison(self, show: bool = True) -> Optional[str]:
        """
        Genera gráficos comparativos (requiere matplotlib).
        
        Args:
            show: Si True, muestra los gráficos
            
        Returns:
            Path al archivo de imagen si se guardó
        """
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            logger.warning("matplotlib no disponible para gráficos")
            return None
        
        if not self.records:
            logger.warning("No hay registros para graficar")
            return None
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Comparativa: Control Cuántico vs Clásico', fontsize=16)
        
        # Extrae datos
        times = [r.timestamp for r in self.records]
        
        # Subplot 1: Error de posición
        if self.quantum_records:
            q_times = [r.timestamp for r in self.quantum_records]
            q_errors = [r.position_error for r in self.quantum_records]
            axes[0, 0].plot(q_times, q_errors, 'b-', label='Cuántico (QAOA)', alpha=0.7)
        
        if self.classical_records:
            c_times = [r.timestamp for r in self.classical_records]
            c_errors = [r.position_error for r in self.classical_records]
            axes[0, 0].plot(c_times, c_errors, 'r--', label='Clásico (PID)', alpha=0.7)
        
        axes[0, 0].set_xlabel('Tiempo (s)')
        axes[0, 0].set_ylabel('Error de Posición (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # Subplot 2: Control input
        if self.quantum_records:
            q_times = [r.timestamp for r in self.quantum_records]
            q_control = [r.control_input for r in self.quantum_records]
            axes[0, 1].plot(q_times, q_control, 'b-', label='Cuántico', alpha=0.7)
        
        if self.classical_records:
            c_times = [r.timestamp for r in self.classical_records]
            c_control = [r.control_input for r in self.classical_records]
            axes[0, 1].plot(c_times, c_control, 'r--', label='Clásico', alpha=0.7)
        
        axes[0, 1].set_xlabel('Tiempo (s)')
        axes[0, 1].set_ylabel('Control (V)')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # Subplot 3: Costo función
        if self.quantum_records:
            q_times = [r.timestamp for r in self.quantum_records]
            q_costs = [r.cost_function for r in self.quantum_records]
            axes[1, 0].plot(q_times, q_costs, 'b-', label='Cuántico', alpha=0.7)
        
        if self.classical_records:
            c_times = [r.timestamp for r in self.classical_records]
            c_costs = [r.cost_function for r in self.classical_records]
            axes[1, 0].plot(c_times, c_costs, 'r--', label='Clásico', alpha=0.7)
        
        axes[1, 0].set_xlabel('Tiempo (s)')
        axes[1, 0].set_ylabel('Función de Costo J(x,u)')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        # Subplot 4: Velocidad del motor
        if self.records:
            velocities = [r.motor_omega for r in self.records]
            axes[1, 1].plot(times, velocities, 'g-', label='ω motor', alpha=0.7)
        
        axes[1, 1].set_xlabel('Tiempo (s)')
        axes[1, 1].set_ylabel('Velocidad Angular (rad/s)')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Guardar figura
        output_file = self.output_dir / f"comparison_{self.session_id}.png"
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        logger.info(f"Gráfico guardado en {output_file}")
        
        if show:
            plt.show()
        
        plt.close()
        return str(output_file)
