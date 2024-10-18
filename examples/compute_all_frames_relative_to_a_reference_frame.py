import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

from coordinates.frame_manager import FrameManager
import numpy as np

# Example usage
if __name__ == "__main__":
    manager = FrameManager()

    # Initialize frames
    manager.initialize_frames(["A", "B", "C", "D"])
    
    # Print the initialized frames, with None as the transformation matrix
    manager.print_frames()

    # Add transformations (A as the reference frame)
    T_A_to_B = np.array([[1, 0, 0, 1],
                         [0, 1, 0, 2],
                         [0, 0, 1, 3],
                         [0, 0, 0, 1]])
    T_B_to_C = np.array([[0, -1, 0, 0],
                         [1,  0, 0, 1],
                         [0,  0, 1, 1],
                         [0,  0, 0, 1]])
    T_C_to_D = np.array([[1, 0, 0, 2],
                         [0, 1, 0, 1],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

    # Register transformations
    manager.add_transformation("A", "B", T_A_to_B)
    manager.add_transformation("B", "C", T_B_to_C)
    manager.add_transformation("C", "D", T_C_to_D)

    # add a known reference frame, otherwise compute_all_frames will not work
    manager.add_frame("A", np.eye(4))
    
    # Compute all frames relative to A
    print("\nComputing all frames relative to frame A:")
    manager.compute_all_frames("A")
    
    # Print all frames
    manager.print_frames()
    
    # Print all transformations
    manager.print_transformations()

    # Visualize all frames
    manager.visualize_known_frames()