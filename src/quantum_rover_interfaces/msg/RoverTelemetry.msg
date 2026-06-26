# Full telemetry record emitted once per control cycle by the hybrid controller.
# Mirrors the fields logged for offline quantum-vs-classical benchmarking.
std_msgs/Header header

uint64 iteration                 # Monotonic control-loop counter

# --- Plant state ---------------------------------------------------------
float64 position                 # Linear position of the rover body (m)
float64 velocity                 # Linear velocity of the rover body (m/s)
MotorState motor                 # Drive-motor electromechanical state

# --- Control signal ------------------------------------------------------
float64 control_input            # Commanded motor voltage / PWM (V)
string  control_source           # Active controller: "quantum" | "classical"

# --- Tracking error ------------------------------------------------------
float64 position_error           # reference_position - position (m)
float64 velocity_error           # reference_omega - omega (rad/s)

# --- Dynamics / cost -----------------------------------------------------
float64 tau_slip                 # Torque lost to wheel slip / traction loss (N*m)
float64 cost_function            # LQR cost J = x^T Q x + u^T R u

# --- Optimizer diagnostics ----------------------------------------------
float64 optimization_time_ms     # Wall-clock time spent in optimize() (ms)
bool    quantum_backend_ok       # True if the quantum backend executed this cycle
