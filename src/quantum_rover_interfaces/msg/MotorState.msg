# Instantaneous electromechanical state of a single DC drive motor.
std_msgs/Header header

float64 omega          # Angular velocity of the shaft (rad/s)
float64 domega_dt      # Angular acceleration (rad/s^2)
float64 current        # Armature current (A)
float64 torque         # Electromagnetic torque produced (N*m)
float64 voltage        # Applied terminal voltage / PWM command (V)
