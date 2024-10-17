import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform

# 1. Camera Local Frame: At origin, with default orientation (X-right, Y-down, Z-forward)
camera_local_frame = np.eye(4)  # Identity matrix: origin and aligned with global axes

# 2. Camera Frame in World: Translated and rotated to a specific position in the world
translation = [2, 3, 1]  # Camera's position in the world
theta = np.radians(30)  # 30-degree rotation about Z-axis
rotation_z = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0, 0, 1]
])

# Create the 4x4 transformation matrix for the camera in the world
camera_in_world_frame = np.eye(4)
camera_in_world_frame[:3, :3] = rotation_z  # Insert rotation
camera_in_world_frame[:3, 3] = translation  # Insert translation

# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the camera's local frame (camera perspective)
plot_transform(ax=ax, A2B=camera_local_frame, name="Camera Local Frame", s=0.5)

# Plot the camera's frame in the world coordinate system
plot_transform(ax=ax, A2B=camera_in_world_frame, name="Camera in World", s=0.5)

# Customize plot limits
ax.set_xlim([-1, 5])
ax.set_ylim([-1, 5])
ax.set_zlim([-1, 3])

# Add labels and title
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
plt.title("Camera Local Frame and Camera Frame in World")

# Show plot
plt.show()
