import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform
from pytransform3d.rotations import random_vector
from scipy.spatial.transform import Rotation as R

# Initialize the 3D figure and axis
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim([-3, 3])
ax.set_ylim([-3, 3])
ax.set_zlim([-3, 3])

# Add reference frames (optional)
plot_transform(ax=ax, A2B=np.eye(4), name="Start Frame")  # Frame at origin

# Function to generate new transformation in each iteration
def get_new_transformation():
    # Generate a random rotation and a small translation as an example
    rotation = R.from_rotvec(random_vector()).as_matrix()
    translation = np.random.uniform(-1, 1, size=3)

    # Create a 4x4 transformation matrix
    transformation = np.eye(4)
    transformation[:3, :3] = rotation
    transformation[:3, 3] = translation
    return transformation

# Loop to update the transformation dynamically
for _ in range(100):  # Example with 100 iterations
    # Get the new transformation matrix in each iteration
    new_transform = get_new_transformation()

    # Clear the previous plot
    ax.cla()
    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([-3, 3])

    # Re-plot the initial frame and the new dynamic frame
    plot_transform(ax=ax, A2B=np.eye(4), name="Start Frame")  # Frame at origin
    plot_transform(ax=ax, A2B=new_transform, name="Dynamic Frame")  # New frame

    # Pause to create the animation effect
    plt.pause(0.1)

plt.show()
