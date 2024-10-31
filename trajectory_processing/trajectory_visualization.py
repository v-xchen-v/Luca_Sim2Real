import numpy as np
import matplotlib.pyplot as plt

def plot_trajectory_xyz_over_steps(trajectory: np.darray):
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