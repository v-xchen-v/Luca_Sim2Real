import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform

# Helper function to create a 4x4 transformation matrix
def create_transformation_matrix(translation, rotation_matrix=None):
    T = np.eye(4)  # Initialize identity matrix
    if rotation_matrix is not None:
        T[:3, :3] = rotation_matrix  # Set rotation
    T[:3, 3] = translation  # Set translation
    return T

# Example: Rotation matrix (board tilted by 15 degrees around X-axis)
theta = np.radians(15)
rotation_x = np.array([
    [1, 0, 0],
    [0, np.cos(theta), -np.sin(theta)],
    [0, np.sin(theta), np.cos(theta)]
])

# 1. Calibration Board Local Frame: At origin with no rotation
calibration_board_local_frame = create_transformation_matrix([0, 0, 0])

# 2. Calibration Board Real Frame: Translated to (1, 1, 0.5) and rotated 15 degrees
calibration_board_real_frame = create_transformation_matrix([1, 1, 0.5], rotation_x)

# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot frames
plot_transform(ax=ax, A2B=calibration_board_local_frame, name="Calibration Board Local", s=0.3)
plot_transform(ax=ax, A2B=calibration_board_real_frame, name="Calibration Board Real", s=0.3)

# Customize plot limits and labels
ax.set_xlim([-1, 3])
ax.set_ylim([-1, 3])
ax.set_zlim([-1, 2])

ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
plt.title("Calibration Board: Local vs Real Frame")

# Show plot
plt.show()
