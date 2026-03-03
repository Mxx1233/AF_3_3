# Motor control command
# Published by controller_node to uc_bridge_adapter_node
std_msgs/Header header

# Steering angle [rad]
# Desired steering angle of the servo
# Positive = turn right
# Negative = turn left
# Recommended range: [-0.52, 0.52] (approx. ±30 degrees)
float32 steering_angle

# Motor control level
# Normalized motor drive signal, range [-1.0, 1.0]
# (0.0, 1.0]  -> forward (throttle)
#  0.0        -> stop / neutral
# [-1.0, 0.0) -> reverse or active braking
# Note: phase 1 uses only the [0.0, 1.0] range
float32 motor_level
