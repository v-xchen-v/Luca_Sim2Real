import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform

# Helper function to create a transformation matrix
def create_transformation_matrix(translation, rotation_matrix=None):
    T = np.eye(4)  # Initialize 4x4 identity matrix
    if rotation_matrix is not None:
        T[:3, :3] = rotation_matrix  # Set rotation matrix
    T[:3, 3] = translation  # Set translation vector
    return T

# Example: 30-degree rotation around Z-axis
theta = np.radians(30)
rotation_z = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0, 0, 1]
])

# Robot Base Local Frame: Located at origin, no rotation
robot_base_local_frame = create_transformation_matrix([0, 0, 0])

# Robot Base Real Frame: Positioned at (2, 2, 0) in the real world, rotated by 30 degrees
robot_base_real_frame = create_transformation_matrix([2, 2, 0], rotation_z)

# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the Robot Base Local Frame
plot_transform(ax=ax, A2B=robot_base_local_frame, name="Robot Base Local", s=0.5)

# Plot the Robot Base Real Frame
plot_transform(ax=ax, A2B=robot_base_real_frame, name="Robot Base Real", s=0.5)

# Customize plot limits and labels
ax.set_xlim([-1, 4])
ax.set_ylim([-1, 4])
ax.set_zlim([-1, 2])

ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
plt.title("Robot Base: Local Frame vs. Real Frame")

# Show the plot
plt.show()
