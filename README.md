<div align="center">

# 🛰️ Hybrid Quantum-Classical Actuator Control — ROS 2 Stack

**A managed ROS 2 control stack for an autonomous rover that closes its
velocity loop with a QAOA quantum optimizer and degrades gracefully to a tuned
PID controller.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org)
[![CI](https://github.com/deimosvn/Hybrid-Quantum-Actuator-Control/actions/workflows/ci.yml/badge.svg)](https://github.com/deimosvn/Hybrid-Quantum-Actuator-Control/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)
[![Quantum: Qiskit](https://img.shields.io/badge/quantum-Qiskit%20%7C%20Aer-6929C4?logo=qiskit&logoColor=white)](https://www.ibm.com/quantum/qiskit)

📖 **English** · [Español](README.es.md)

</div>

---

## Overview

This repository turns a quantum-classical control *experiment* into a clean,
buildable **ROS 2 workspace**. A simulated rover (DC motor with traction loss)
is driven around a velocity reference by a **lifecycle-managed controller** that
can solve the LQR control problem either with **QAOA** (Quantum Approximate
Optimization Algorithm, on the Qiskit Aer simulator) or with a classical
**PID** fallback — and stream rich telemetry so the two can be benchmarked.

It is built to look and behave like a real robotics project: custom interfaces,
QoS-aware topics, services and an action, TF + odometry, a URDF you can open in
RViz, parameter files, launch orchestration, unit + lint tests, CI, and a
container for one-command reproducibility.

> 📚 The control theory (QAOA circuit, LQR cost, motor model) lives in
> [`docs/THEORY.md`](docs/THEORY.md). The system design lives in
> [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

## Features

- 🧠 **Hybrid controller** — QAOA optimizer with an automatic, transparent PID
  fallback (the quantum backend is *optional*).
- ♻️ **Lifecycle node** — deterministic `configure → activate` bring-up; no
  command is published while inactive.
- 🛰️ **Standard ROS interfaces** — publishes `nav_msgs/Odometry`,
  `sensor_msgs/JointState`, broadcasts TF, and emits `diagnostic_msgs`.
- 🧩 **Custom interfaces** — `RoverTelemetry`/`MotorState` messages, a
  `SetControlMode`/`OptimizeControl` service pair, and a
  `FollowVelocityProfile` action with live feedback + metrics.
- 🎛️ **Declared parameters** with ranges/descriptors, loadable from YAML.
- 📊 **Telemetry logging** to CSV/JSON + comparison plots and RMSE/IAE/ISE
  metrics.
- 🤖 **URDF + RViz** four-wheel rover description.
- ✅ **Tested & linted** — pytest unit tests plus `ament_flake8`,
  `ament_pep257`, `ament_copyright`; multi-distro GitHub Actions CI.
- 🐳 **Dockerfile + devcontainer** so it builds the same on Windows/macOS/Linux.

## Repository structure

```
Hybrid-Quantum-Actuator-Control/
├── src/
│   ├── quantum_rover_interfaces/     # msg / srv / action  (ament_cmake)
│   ├── quantum_rover_control/        # algorithms + rclpy nodes (ament_python)
│   │   └── quantum_rover_control/
│   │       ├── core/                 # DC-motor physics + QAOA/PID optimizers
│   │       ├── utils/                # telemetry dataclass + DataLogger
│   │       ├── nodes/                # simulator / controller / logger nodes
│   │       └── sim_demo.py           # standalone (no-ROS) simulation
│   ├── quantum_rover_description/    # URDF/xacro + RViz (ament_cmake)
│   └── quantum_rover_bringup/        # launch + parameter sets (ament_cmake)
├── docs/                             # ARCHITECTURE.md, THEORY.md, QUICKSTART_ES.md
├── docker/ · Dockerfile · docker-compose.yml · .devcontainer/
├── .github/workflows/ci.yml
└── LICENSE
```

## Quick start

### Option A — Docker (recommended, works on any OS)

```bash
git clone https://github.com/deimosvn/Hybrid-Quantum-Actuator-Control.git
cd Hybrid-Quantum-Actuator-Control
docker compose build
# Headless closed-loop simulation:
docker compose run --rm rover ros2 launch quantum_rover_bringup simulation.launch.py
```

### Option B — Native ROS 2 (Ubuntu 22.04 / Humble or 24.04 / Jazzy)

```bash
# 0) Source your ROS 2 installation
source /opt/ros/humble/setup.bash

# 1) Create a workspace and add this repo to src/
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/deimosvn/Hybrid-Quantum-Actuator-Control.git
cd ~/ros2_ws

# 2) Install dependencies
rosdep install --from-paths src --ignore-src -r -y
# Optional quantum backend (PID fallback is used if absent):
pip install "qiskit>=1.0" "qiskit-aer>=0.14"

# 3) Build & source
colcon build --symlink-install
source install/setup.bash
```

> The cloned folder already contains `src/`; you can also build it directly as a
> workspace root instead of nesting it under another `src/`.

## Running it

**Full system with visualization (RViz):**

```bash
ros2 launch quantum_rover_bringup quantum_rover.launch.py
# Pick the backend:
ros2 launch quantum_rover_bringup quantum_rover.launch.py use_quantum:=false
```

**Headless benchmark run (for CI / data collection):**

```bash
ros2 launch quantum_rover_bringup simulation.launch.py use_quantum:=true
```

**Drive a velocity setpoint manually:**

```bash
ros2 topic pub /reference_omega std_msgs/Float64 "{data: 8.0}"
```

**Switch controller backend at runtime:**

```bash
ros2 service call /quantum_controller/set_control_mode \
  quantum_rover_interfaces/srv/SetControlMode "{mode: 'classical'}"
```

**Single-shot optimization query:**

```bash
ros2 service call /quantum_controller/optimize_control \
  quantum_rover_interfaces/srv/OptimizeControl "{position_error: 0.0, velocity_error: 3.0}"
```

**Run a benchmark velocity profile (action with live feedback + metrics):**

```bash
ros2 action send_goal --feedback /quantum_controller/follow_velocity_profile \
  quantum_rover_interfaces/action/FollowVelocityProfile \
  "{target_omega: [5.0, 8.0, 3.0], hold_duration: [4.0, 4.0, 4.0], settle_tolerance: 0.3}"
```

**Inspect the running graph:**

```bash
ros2 topic echo /telemetry              # quantum_rover_interfaces/RoverTelemetry
ros2 topic echo /diagnostics
ros2 lifecycle get /quantum_controller
ros2 run rqt_graph rqt_graph
```

## Parameters

Defaults live in
[`config/control_params.yaml`](src/quantum_rover_control/config/control_params.yaml);
override any of them per launch with `params_file:=...`.

| Node | Parameter | Default | Description |
|------|-----------|---------|-------------|
| `quantum_controller` | `use_quantum` | `true` | Start in QAOA mode (else PID). |
| | `num_qubits` / `num_layers` | `2` / `1` | QAOA register width / depth. |
| | `control_frequency` | `100.0` | Control-loop rate (Hz). |
| | `pwm_max` | `12.0` | Voltage saturation (V). |
| | `q_position_weight` / `q_velocity_weight` / `r_control_weight` | `10 / 1 / 0.5` | LQR weights. |
| `rover_simulator` | `sim_frequency` | `1000` | Physics integration rate (Hz). |
| | `publish_frequency` | `100.0` | State publication rate (Hz). |
| | `rover.traction_loss` | `0.2` | Fraction of traction lost to slip. |
| | `motor.*` | — | DC-motor constants (K_t, K_e, R, J, b…). |
| `telemetry_logger` | `output_dir` | `./logs` | Where CSV/JSON/PNG are written. |

## Standalone demo (no ROS 2 required)

The control algorithms run without ROS for quick algorithm validation:

```bash
# After building/sourcing:
ros2 run quantum_rover_control sim_demo
# …or as a plain Python module:
python -m quantum_rover_control.sim_demo
```

## Testing

```bash
colcon test --packages-select quantum_rover_control --event-handlers console_direct+
colcon test-result --verbose
```

This runs the algorithm unit tests
([`test/test_core_algorithms.py`](src/quantum_rover_control/test/test_core_algorithms.py))
and the `flake8` / `pep257` / `copyright` linters.

## Documentation

| Doc | Contents |
|-----|----------|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Packages, node graph, interfaces, design rationale. |
| [docs/THEORY.md](docs/THEORY.md) | QAOA, LQR, DC-motor math, full API reference. |
| [docs/QUICKSTART_ES.md](docs/QUICKSTART_ES.md) | Guía rápida en español (versión original). |

## Roadmap

- [ ] `ros2_control` hardware interface for sim-to-real hand-off.
- [ ] Gazebo / Ignition world for full 3D dynamics.
- [ ] Run QAOA on real IBM Quantum hardware via `qiskit-ibm-runtime`.
- [ ] Kalman state estimator node between plant and controller.

## Contributing

Issues and PRs are welcome. Please keep `colcon test` green (unit tests + ament
linters) and follow the existing node/parameter conventions.

## License

Released under the [MIT License](LICENSE).

## Author & citation

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
