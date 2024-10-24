import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform
import numpy as np

def visualize_frame(T, name):
    """
    Visualize a single coordinate frame in 3D space.
    
    Parameters:
    - T: 4x4 transformation matrix for the frame.
    - name: Name of the frame.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    plot_transform(ax=ax, A2B=T, name=name, s=0.2)
    
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    
    plt.title(f"3D Visualization of {name} Frame")
    plt.show()
    
    
def visualize_frames(frames, frame_names, limits=None):
    """
    Visualize a set of coordinate frames in 3D space.
    
    Parameters:
    - frames: List of 4x4 transformation matrices for the frames to visualize.
    - frame_names: List of names for the frames.
    - limits: Tuple of (xlim, ylim, zlim) for axis limits, optional.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot each frame in the list
    for T, name in zip(frames, frame_names):
        plot_transform(ax=ax, A2B=T, name=name, s=0.2)
    
    # Set custom axis limits if provided
    if limits:
        ax.set_xlim(limits[0])
        ax.set_ylim(limits[1])
        ax.set_zlim(limits[2])
    else:
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
    
    # Set axis labels
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    
    plt.title("3D Visualization of Coordinate Frames")
    plt.show()

def visualize_robot_with_objects(T_robot_base, T_camera, T_object_real, T_calibration_board, limits=None):
    """
    Visualize the robot base, camera, object, and calibration board frames.
    
    Parameters:
    - T_robot_base: 4x4 transformation matrix for the robot base frame.
    - T_camera: 4x4 transformation matrix for the camera frame.
    - T_object_real: 4x4 transformation matrix for the object in the real world.
    - T_calibration_board: 4x4 transformation matrix for the calibration board.
    - limits: Tuple of (xlim, ylim, zlim) for axis limits, optional.
    """
    frames = [T_robot_base, T_camera, T_object_real, T_calibration_board]
    frame_names = ["robot_base", "camera", "object_real", "calibration_board"]
    
    visualize_frames(frames, frame_names, limits)

def visualize_transformation(T, from_frame, to_frame):
    """
    Visualize a transformation between two coordinate frames.
    
    Parameters:
    - T: 4x4 transformation matrix describing the transformation.
    - from_frame: The name of the starting frame.
    - to_frame: The name of the target frame.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the from_frame at origin
    plot_transform(ax=ax, A2B=np.eye(4), name=from_frame, s=0.2)
    
    # Plot the transformed to_frame
    plot_transform(ax=ax, A2B=T, name=to_frame, s=0.2)
    
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    
    plt.title(f"Transformation: {from_frame} to {to_frame}")
    plt.show()

def visualize_robot_base_to_world(T_robot_base_real, T_robot_base_sim, T_world_real):
    """
    Visualize both the real and simulated robot base frames relative to the world frame.
    
    Parameters:
    - T_robot_base_real: 4x4 transformation matrix for the real-world robot base.
    - T_robot_base_sim: 4x4 transformation matrix for the simulated robot base.
    - T_world_real: 4x4 transformation matrix for the world frame (real-world reference).
    """
    frames = [T_world_real, T_robot_base_real, T_robot_base_sim]
    frame_names = ["world_real", "robot_base_real", "robot_base_sim"]
    
    visualize_frames(frames, frame_names)

