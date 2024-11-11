import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib
from typing import Callable
# matplotlib.use('TkAgg')  # Or 'Qt5Agg', depending on your setup

def plot_trajectory_xyz_over_steps(trajectory: np.ndarray):
    """
    Plot the trajectory of x, y, z over steps.
    """
    plt.plot(trajectory[:, 0], label='x')
    plt.plot(trajectory[:, 1], label='y')
    plt.plot(trajectory[:, 2], label='z')
    plt.xlabel('Step')
    plt.ylabel('Value')
    plt.legend()
    plt.show()  
    
    
def animate_3d_transformation_over_steps(num_transformation_steps: int, 
                                         visualization_func: Callable[[plt.Axes, int], None],
                                         interval: int=100, 
                                         limits: list=[[-1, 1], [-1, 1], [-1, 1]]):
    """
    Animates a sequence of transformations in 3D.

    Parameters:
        num_transformation_steps (int): How many transformation steps we want to animate.
        visualization_func (function): Function to prepare and visualize frames.
        interval (int): Interval between frames in milliseconds.
        limits (list of list of float): Axis limits for [x_min, x_max], [y_min, y_max], [z_min, z_max].
    """
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(limits[0])
    ax.set_ylim(limits[1])
    ax.set_zlim(limits[2])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    
    
    def update_frame(i):
        ax.cla()
        ax.set_xlim(limits[0])
        ax.set_ylim(limits[1])
        ax.set_zlim(limits[2])
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        
        
        # Call the passed visualization function
        visualization_func(ax, i)
        
    anim = FuncAnimation(fig, update_frame, frames=num_transformation_steps, interval=interval)
    plt.show(block=True)