import numpy as np
from pytransform3d.transformations import plot_transform
from matplotlib import pyplot as plt

# Define the 3x3 rotation matrix (e.g., 45 degrees around the Z-axis)
theta = np.radians(45)  # 45-degree rotation
rotation_z = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0, 0, 1]
])

# Define the translation vector (e.g., calibration board is 1 meter in front of the camera)
translation = np.array([0, 0, 1])

# Create the 4x4 transformation matrix from camera_real to calibration_board_real
T_camera_to_board = np.eye(4)  # Initialize with identity matrix
T_camera_to_board[:3, :3] = rotation_z  # Insert rotation matrix
T_camera_to_board[:3, 3] = translation  # Insert translation vector

# Print the transformation matrix
print("Transformation Matrix (Camera -> Calibration Board):")
print(T_camera_to_board)

# Visualization of the transformation
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the camera frame (camera_real) at the origin
plot_transform(ax=ax, A2B=np.eye(4), name="camera_real", s=0.2)

# Plot the calibration board frame (calibration_board_real) after applying the transformation
plot_transform(ax=ax, A2B=T_camera_to_board, name="calibration_board_real", s=0.2)

# Customize plot limits and labels
ax.set_xlim([-1, 2])
ax.set_ylim([-1, 2])
ax.set_zlim([-1, 2])

ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
plt.title("Transformation: Camera to Calibration Board")

# Show the plot
plt.show()
