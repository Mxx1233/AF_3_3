# Lane deviation information
# Published by lane_detection_node to controller_node

# Standard message header (contains timestamp and coordinate frame ID)
std_msgs/Header header

# Lateral error [m]
# Description: lateral displacement of the vehicle reference point relative to the lane center
# Positive = vehicle is to the right of the lane center
# Negative = vehicle is to the left of the lane center
float32 lateral_error

# Heading error [rad]
# Description: angle between the vehicle heading and the lane tangent direction
# Positive = vehicle heading points to the right of the lane
# Negative = vehicle heading points to the left of the lane
float32 heading_error

# Lane curvature [1/m]
# Description: curvature of the current lane
# 0 = straight
# Positive = right curve
# Negative = left curve
float32 curvature
