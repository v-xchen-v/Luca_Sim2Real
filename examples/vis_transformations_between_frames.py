"""to visualize transformations between frames using only the 4x4 transformation matrix, you can do it by visualizing the 
original frame and then applying successive transformations to create new frames from the same origin. 
This will show how one frame transforms into another.
"""

import numpy as np
from pytransform3d.transformations import plot_transform, concat
import matplotlib.pyplot as plt

# Create two transformation matrices (T1 and T2)
T1 = np.array([
    [1.0, 0.0, 0.0, 1.0],  # Identity rotation with translation [1, 0, 0]
    [0.0, 1.0, 0.0, 2.0],
    [0.0, 0.0, 1.0, 3.0],
    [0.0, 0.0, 0.0, 1.0]
])

T2 = np.array([
    [0.0, -1.0, 0.0, 2.0],  # 90 degrees rotation around Z with translation [2, 0, 0]
    [1.0,  0.0, 0.0, 0.0],
    [0.0,  0.0, 1.0, 1.0],
    [0.0,  0.0, 0.0, 1.0]
])

# Combine transformations if you want to visualize cumulative transformations
T_combined = concat(T1, T2)  # T1 * T2

# Create 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot the origin frame
plot_transform(ax, np.eye(4), s=0.2, name='Origin')

# Plot first transformed frame (T1)
plot_transform(ax, T1, s=0.2, name='Frame 1')

# Plot second transformed frame (T2, relative to Origin)
plot_transform(ax, T2, s=0.2, name='Frame 2')

# Plot cumulative transformed frame (T1 * T2)
plot_transform(ax, T_combined, s=0.2, name='Combined')

# Set plot limits for better visualization
ax.set_xlim([-1, 4])
ax.set_ylim([-1, 4])
ax.set_zlim([0, 5])

plt.show()
