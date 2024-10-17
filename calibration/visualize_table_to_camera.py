import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform
from .calibration_data_loader import load_table_to_camera_extrinsics_from_npy

def visualize_table_to_camera(T_table_to_camera):
    """
    Visualize the table-to-camera transformation in 3D space.

    Parameters:
    - T_table_to_camera: 4x4 transformation matrix (table to camera).
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Plot the camera frame at the origin
    plot_transform(ax=ax, A2B=np.eye(4), name="Camera", s=0.2)

    # Plot the table (calibration board) frame after applying the transformation
    plot_transform(ax=ax, A2B=T_table_to_camera, name="Table", s=0.2)

    # Set plot limits and labels
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")

    plt.title("Table to Camera Transformation")
    plt.show()