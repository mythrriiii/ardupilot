import numpy as np
from scipy.spatial.transform import Rotation as R

# Function to convert yaw (in degrees) to quaternion
def yaw_to_quaternion(yaw_deg):
    # Assuming roll and pitch are 0
    return R.from_euler('zyx', [yaw_deg, 0, 0], degrees=True).as_quat()

# 1. Initial position: Facing North (yaw = 0)
start_quat = yaw_to_quaternion(0)

# 2. Hovering 10m South at 15mph: same orientation (still facing North)
hover_quat = yaw_to_quaternion(0)

# 3. Turn to the East (yaw = 90 degrees)
turn_quat = yaw_to_quaternion(90)

# 4. Fly East at 15mph for 50m: maintain same orientation (East)
east_quat = yaw_to_quaternion(90)

# Display quaternions
print("Start quaternion (facing North):", start_quat)
print("Hover quaternion (South of start, facing North):", hover_quat)
print("Turn quaternion (facing East):", turn_quat)
print("East flight quaternion (flying East):", east_quat)
