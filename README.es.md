<div align="center">

# 🛰️ Control de Actuador Híbrido Cuántico-Clásico — Stack de ROS 2

**Un stack de control en ROS 2 para un rover autónomo que cierra su lazo de
velocidad con un optimizador cuántico QAOA y degrada de forma elegante a un
controlador PID clásico.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org)
[![CI](https://github.com/deimosvn/Hybrid-Quantum-Actuator-Control/actions/workflows/ci.yml/badge.svg)](https://github.com/deimosvn/Hybrid-Quantum-Actuator-Control/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)
[![Quantum: Qiskit](https://img.shields.io/badge/quantum-Qiskit%20%7C%20Aer-6929C4?logo=qiskit&logoColor=white)](https://www.ibm.com/quantum/qiskit)

[English](README.md) · 📖 **Español**

</div>

---

## Descripción general

Este repositorio convierte un *experimento* de control cuántico-clásico en un
**workspace de ROS 2** limpio y compilable. Un rover simulado (motor DC con
pérdida de tracción) sigue una referencia de velocidad gracias a un
**controlador gestionado por ciclo de vida (lifecycle)** que puede resolver el
problema de control LQR con **QAOA** (Quantum Approximate Optimization
Algorithm, sobre el simulador Qiskit Aer) o con un **PID** clásico de respaldo,
transmitiendo telemetría rica para comparar ambos enfoques.

Está construido para verse y comportarse como un proyecto de robótica real:
interfaces propias, tópicos con QoS, servicios y una acción, TF + odometría, un
URDF que puedes abrir en RViz, archivos de parámetros, orquestación con launch,
pruebas unitarias + linters, CI y un contenedor para reproducibilidad en un solo
comando.

> 📚 La teoría de control (circuito QAOA, costo LQR, modelo del motor) está en
> [`docs/THEORY.md`](docs/THEORY.md). El diseño del sistema está en
> [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

## Características

- 🧠 **Controlador híbrido** — optimizador QAOA con respaldo PID automático y
  transparente (el backend cuántico es *opcional*).
- ♻️ **Nodo lifecycle** — arranque determinista `configure → activate`; no se
  publica ningún comando mientras está inactivo.
- 🛰️ **Interfaces estándar de ROS** — publica `nav_msgs/Odometry`,
  `sensor_msgs/JointState`, emite TF y `diagnostic_msgs`.
- 🧩 **Interfaces propias** — mensajes `RoverTelemetry`/`MotorState`, el par de
  servicios `SetControlMode`/`OptimizeControl` y una acción
  `FollowVelocityProfile` con feedback en vivo + métricas.
- 🎛️ **Parámetros declarados** con rangos/descriptores, cargables desde YAML.
- 📊 **Registro de telemetría** a CSV/JSON + gráficos comparativos y métricas
  RMSE/IAE/ISE.
- 🤖 **URDF + RViz** del rover de cuatro ruedas.
- ✅ **Probado y con linters** — pruebas unitarias con pytest más
  `ament_flake8`, `ament_pep257`, `ament_copyright`; CI multi-distro en GitHub
  Actions.
- 🐳 **Dockerfile + devcontainer** para que compile igual en Windows/macOS/Linux.

## Estructura del repositorio

```
Hybrid-Quantum-Actuator-Control/
├── src/
│   ├── quantum_rover_interfaces/     # msg / srv / action  (ament_cmake)
│   ├── quantum_rover_control/        # algoritmos + nodos rclpy (ament_python)
│   │   └── quantum_rover_control/
│   │       ├── core/                 # física del motor DC + optimizadores QAOA/PID
│   │       ├── utils/                # dataclass de telemetría + DataLogger
│   │       ├── nodes/                # nodos simulador / controlador / logger
│   │       └── sim_demo.py           # simulación standalone (sin ROS)
│   ├── quantum_rover_description/    # URDF/xacro + RViz (ament_cmake)
│   └── quantum_rover_bringup/        # launch + juegos de parámetros (ament_cmake)
├── docs/                             # ARCHITECTURE.md, THEORY.md, QUICKSTART_ES.md
├── docker/ · Dockerfile · docker-compose.yml · .devcontainer/
├── .github/workflows/ci.yml
└── LICENSE
```

## Inicio rápido

### Opción A — Docker (recomendada, funciona en cualquier SO)

```bash
git clone https://github.com/deimosvn/Hybrid-Quantum-Actuator-Control.git
cd Hybrid-Quantum-Actuator-Control
docker compose build
# Simulación de lazo cerrado sin interfaz gráfica:
docker compose run --rm rover ros2 launch quantum_rover_bringup simulation.launch.py
```

### Opción B — ROS 2 nativo (Ubuntu 22.04 / Humble o 24.04 / Jazzy)

```bash
# 0) Carga tu instalación de ROS 2
source /opt/ros/humble/setup.bash

# 1) Crea un workspace y añade este repo a src/
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/deimosvn/Hybrid-Quantum-Actuator-Control.git
cd ~/ros2_ws

# 2) Instala dependencias
rosdep install --from-paths src --ignore-src -r -y
# Backend cuántico opcional (si falta, se usa el PID de respaldo):
pip install "qiskit>=1.0" "qiskit-aer>=0.14"

# 3) Compila y carga el entorno
colcon build --symlink-install
source install/setup.bash
```

> La carpeta clonada ya contiene `src/`; también puedes compilarla directamente
> como raíz del workspace en lugar de anidarla bajo otro `src/`.

## Ejecución

**Sistema completo con visualización (RViz):**

```bash
ros2 launch quantum_rover_bringup quantum_rover.launch.py
# Elige el backend:
ros2 launch quantum_rover_bringup quantum_rover.launch.py use_quantum:=false
```

**Ejecución sin GUI (para CI / recolección de datos):**

```bash
ros2 launch quantum_rover_bringup simulation.launch.py use_quantum:=true
```

**Comandar una referencia de velocidad manualmente:**

```bash
ros2 topic pub /reference_omega std_msgs/Float64 "{data: 8.0}"
```

**Cambiar el backend del controlador en caliente:**

```bash
ros2 service call /quantum_controller/set_control_mode \
  quantum_rover_interfaces/srv/SetControlMode "{mode: 'classical'}"
```

**Consulta de optimización puntual:**

```bash
ros2 service call /quantum_controller/optimize_control \
  quantum_rover_interfaces/srv/OptimizeControl "{position_error: 0.0, velocity_error: 3.0}"
```

**Ejecutar un perfil de velocidad de benchmark (acción con feedback + métricas):**

```bash
ros2 action send_goal --feedback /quantum_controller/follow_velocity_profile \
  quantum_rover_interfaces/action/FollowVelocityProfile \
  "{target_omega: [5.0, 8.0, 3.0], hold_duration: [4.0, 4.0, 4.0], settle_tolerance: 0.3}"
```

**Inspeccionar el grafo en ejecución:**

```bash
ros2 topic echo /telemetry              # quantum_rover_interfaces/RoverTelemetry
ros2 topic echo /diagnostics
ros2 lifecycle get /quantum_controller
ros2 run rqt_graph rqt_graph
```

## Parámetros

Los valores por defecto están en
[`config/control_params.yaml`](src/quantum_rover_control/config/control_params.yaml);
sobreescribe cualquiera por launch con `params_file:=...`.

| Nodo | Parámetro | Por defecto | Descripción |
|------|-----------|-------------|-------------|
| `quantum_controller` | `use_quantum` | `true` | Arrancar en modo QAOA (si no, PID). |
| | `num_qubits` / `num_layers` | `2` / `1` | Ancho / profundidad del registro QAOA. |
| | `control_frequency` | `100.0` | Frecuencia del lazo de control (Hz). |
| | `pwm_max` | `12.0` | Saturación de voltaje (V). |
| | `q_position_weight` / `q_velocity_weight` / `r_control_weight` | `10 / 1 / 0.5` | Pesos LQR. |
| `rover_simulator` | `sim_frequency` | `1000` | Frecuencia de integración física (Hz). |
| | `publish_frequency` | `100.0` | Frecuencia de publicación de estado (Hz). |
| | `rover.traction_loss` | `0.2` | Fracción de tracción perdida por deslizamiento. |
| | `motor.*` | — | Constantes del motor DC (K_t, K_e, R, J, b…). |
| `telemetry_logger` | `output_dir` | `./logs` | Dónde se escriben CSV/JSON/PNG. |

## Demo standalone (sin ROS 2)

Los algoritmos de control corren sin ROS para validación rápida:

```bash
# Después de compilar/cargar el entorno:
ros2 run quantum_rover_control sim_demo
# …o como módulo de Python plano:
python -m quantum_rover_control.sim_demo
```

## Pruebas

```bash
colcon test --packages-select quantum_rover_control --event-handlers console_direct+
colcon test-result --verbose
```

Esto ejecuta las pruebas unitarias de los algoritmos
([`test/test_core_algorithms.py`](src/quantum_rover_control/test/test_core_algorithms.py))
y los linters `flake8` / `pep257` / `copyright`.

## Documentación

| Documento | Contenido |
|-----------|-----------|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Paquetes, grafo de nodos, interfaces, decisiones de diseño. |
| [docs/THEORY.md](docs/THEORY.md) | QAOA, LQR, matemáticas del motor DC, referencia de API completa. |
| [docs/QUICKSTART_ES.md](docs/QUICKSTART_ES.md) | Guía rápida en español (versión original). |

## Hoja de ruta

- [ ] Interfaz hardware `ros2_control` para la transición sim-a-real.
- [ ] Mundo de Gazebo / Ignition para dinámica 3D completa.
- [ ] Ejecutar QAOA en hardware real de IBM Quantum vía `qiskit-ibm-runtime`.
- [ ] Nodo estimador de estado (Kalman) entre la planta y el controlador.

## Contribuir

Issues y PRs son bienvenidos. Por favor mantén `colcon test` en verde (pruebas
unitarias + linters de ament) y sigue las convenciones de nodos/parámetros
existentes.

## Licencia

Publicado bajo la [Licencia MIT](LICENSE).

## Autor y cita

**Diego Eduardo Martínez Cruz** ([@deimosvn](https://github.com/deimosvn))

```bibtex
@software{martinez_quantum_rover_2026,
  author  = {Mart\'inez Cruz, Diego Eduardo},
  title   = {Hybrid Quantum-Classical Actuator Control for an Autonomous Rover (ROS 2)},
  year    = {2026},
  url     = {https://github.com/deimosvn/Hybrid-Quantum-Actuator-Control},
  license = {MIT}
}
```
