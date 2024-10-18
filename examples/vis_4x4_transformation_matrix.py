import numpy as np
from pytransform3d.rotations import plot_basis
import matplotlib.pyplot as plt

# Example 4x4 transformation matrix (Rotation + Translation)
transformation_matrix = np.array([
    [0.0, -1.0,  0.0,  1.0],  # Rotated around Z by 90 degrees, translated by [1, 2, 3]
    [1.0,  0.0,  0.0,  2.0],
    [0.0,  0.0,  1.0,  3.0],
    [0.0,  0.0,  0.0,  1.0]
])

# Extract the rotation matrix (top-left 3x3 block)
rotation_matrix = transformation_matrix[:3, :3]

# Extract the translation vector (top-right 3x1 block)
translation_vector = transformation_matrix[:3, 3]

# Create a new 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the origin frame (identity matrix)
plot_basis(ax, np.eye(3), np.zeros(3), s=0.2, label='Origin')

# Plot the transformed frame using the extracted rotation and translation
plot_basis(ax, rotation_matrix, translation_vector, s=0.2, label='Transformed')

# Set plot limits for better visualization
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([0, 5])

plt.show()
