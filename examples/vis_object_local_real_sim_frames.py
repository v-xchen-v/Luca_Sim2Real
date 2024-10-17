import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform

# Helper function to create a 4x4 transformation matrix
def create_transformation_matrix(translation, rotation_matrix=None):
    T = np.eye(4)  # Initialize with identity matrix
    if rotation_matrix is not None:
        T[:3, :3] = rotation_matrix  # Set rotation
    T[:3, 3] = translation  # Set translation
    return T

# Define example rotation matrix (30-degree rotation around Z-axis)
theta = np.radians(30)
rotation_z = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0, 0, 1]
])

# Object frames
object_local_frame = create_transformation_matrix([0, 0, 0])  # Local frame at origin
object_real_frame = create_transformation_matrix([1, 2, 0], rotation_z)  # Real-world position and rotation
object_sim_frame = create_transformation_matrix([2, 1, 0])  # Simulated world position, no rotation

# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot frames
plot_transform(ax=ax, A2B=object_local_frame, name="Object Local Frame", s=0.3)
plot_transform(ax=ax, A2B=object_real_frame, name="Object Real Frame", s=0.3)
plot_transform(ax=ax, A2B=object_sim_frame, name="Object Sim Frame", s=0.3)

# Customize plot limits
ax.set_xlim([-1, 3])
ax.set_ylim([-1, 3])
ax.set_zlim([-1, 2])

# Add labels and title
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
plt.title("Object Frames: Local, Real, and Simulated")

# Show plot
plt.show()
