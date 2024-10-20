import numpy as np
from pytransform3d.transformations import (
    transform, invert_transform, concat, plot_transform
)
from matplotlib import pyplot as plt

class FrameManager:
    """A class to manage coordinate frames used in this system
    
    - With known transformation matrices between frames, and one frame as the reference frame, the others can be computed.
    """
    
    
    def __init__(self) -> None:
        """Initialize the FrameManager."""
        # Store frames in a dictionary
        self.frames = {} # keys: frame_name, values: 4x4 transformation matrix or None (default to None if unknown)
        
        # Store known transformation between frames in a dictionary
        self.transformations = {} # keys: (source_frame, target_frame), values: 4x4 transformation matrix

    def initialize_frames(self, frame_names):
        """Initialize frames with known names."""
        for frame_name in frame_names:
            self.frames[frame_name] = None
            
    def add_frame(self, frame_name, matrix):
        """Add a frame with a given transformation matrix.
        
        Parameters:
        - frame_name: A string representing the frame name.
        - matrix: A 4x4 transformation matrix representing the frame.
        """
        if frame_name in self.frames and self.frames[frame_name] is not None:
            print(f"Frame {frame_name} already exists!")
        else:
            self.frames[frame_name] = matrix
            print(f"Added frame '{frame_name}':\n{matrix}")
            
    def get_transformation(self, from_frame_name, to_frame_name):
        """Get the known transformation matrix between two frames. If the transformation is not known, return None.
        
        Parameters:
        - from_frame_name: A string representing the source frame name.
        - to_frame_name: A string representing the target frame name.
        
        Returns:
        - A 4x4 transformation matrix from the source frame to the target frame.
        """
        
        # check if the transformation is known
        if (from_frame_name, to_frame_name) in self.transformations:
            return self.transformations[(from_frame_name, to_frame_name)]
        else:
            # check if the reverse transformation is known
            if (to_frame_name, from_frame_name) in self.transformations:
                return invert_transform(self.transformations[(to_frame_name, from_frame_name)])
            else:
                T = self.compute_transformation(from_frame_name, to_frame_name)
                if T is not None:
                    return T
                else:
                    return invert_transform(self.compute_transformation(to_frame_name, from_frame_name))
    
    # def update_frame(self, frame_name, matrix):
    #     """Update an existing frame with a new transformation matrix."""
    #     if frame_name in self.frames:
    #         self.frames[frame_name] = matrix
    #     else:
    #         print(f"Frame {frame_name} not found!")

    def add_transformation(self, from_frame, to_frame, matrix):
        """Add a transformation between two frames."""
        if from_frame not in self.frames or to_frame not in self.frames:
            raise ValueError(f"Both frames '{from_frame}' and '{to_frame}' must exist.")
        
        # check if matrix is valid
        if matrix.shape != (4, 4):
            raise ValueError(f"Transformation matrix must be a 4x4 matrix.")
        
        if (from_frame, to_frame) in self.transformations:
            print(f"Transformation from {from_frame} to {to_frame} already exists!")
        elif (to_frame, from_frame) in self.transformations:
            print(f"Transformation from {to_frame} to {from_frame} already exists!")
        else:
            self.transformations[(from_frame, to_frame)] = matrix
        
    # def compute_all_frames(self, reference_frame):
    #     """Compute the transformations of all frames relative to a reference frame."""
    #     if reference_frame not in self.frames:
    #         raise ValueError(f"Reference frame '{reference_frame}' does not exist.")
    #     # self.frames[reference_frame] = np.eye(4)  # Reference frame at origin
        
    #     # ensure that the reference frame is known
    #     if self.frames[reference_frame] is None:
    #         raise ValueError(f"Reference frame '{reference_frame}' is unknown.")

    #     # Recursively compute transformations for other frames
    #     visited = set()
    #     self._compute_frame_recursively(reference_frame, visited)

    # def _compute_frame_recursively(self, current_frame, visited):
    #     """Recursively compute transformations relative to the current frame."""
    #     visited.add(current_frame)

    #     for neighbor in self._get_to_neighbors(current_frame):
    #         if neighbor in visited:
    #             continue

    #         if (current_frame, neighbor) in self.transformations:
    #             transformation = self.transformations[(current_frame, neighbor)]
    #         elif (neighbor, current_frame) in self.transformations:
    #             # Use the inverse if the reverse transformation is known
    #             transformation = invert_transform(self.transformations[(neighbor, current_frame)])
    #         else:
    #             # if no transformation is known, raise an error
    #             raise ValueError(f"No transformation found between '{current_frame}' and '{neighbor}'.")

    #         # Compute the neighbor's transformation relative to the reference frame
    #         self.frames[neighbor] = concat(self.frames[current_frame], transformation)
    #         print(f"Computed frame '{neighbor}' relative to '{current_frame}':\n{self.frames[neighbor]}")

    #         # Recurse to compute transformations for other connected frames
    #         self._compute_frame_recursively(neighbor, visited)
        
        

        
    def compute_transformation(self, from_frame, to_frame):
        """Compute the transformation between two frames if possible."""
        if (from_frame, to_frame) in self.transformations:
            return self.transformations[(from_frame, to_frame)]
        elif (to_frame, from_frame) in self.transformations:
            # Use the inverse if the reverse transformation is known
            return invert_transform(self.transformations[(to_frame, from_frame)])
        else:
            # Attempt to find a path and compute the full transformation
            return self._find_path_and_compute(from_frame, to_frame)

    def _find_path_and_compute(self, from_frame, to_frame, visited=None):
        """Find a transformation path and compute it."""
        if visited is None:
            visited = set()
        visited.add(from_frame)

        for next_frame in self._get_to_neighbors(from_frame):
            if next_frame in visited:
                continue
            
            # Recursively find a path to the target frame
            partial_transform = self.compute_transformation(from_frame, next_frame)
            remaining_transform = self.compute_transformation(next_frame, to_frame)

            if remaining_transform is not None:
                # Concatenate transformations using pytransform3d's `concat`
                full_transform = concat(remaining_transform, partial_transform)
                # self.add_transformation(from_frame, to_frame, full_transform)
                return full_transform
        return None

    def _get_to_neighbors(self, frame):
        """Get neighboring frames connected to a given frame."""
        return {
            to for (from_frame, to) in self.transformations if from_frame == frame
        }

    def update_transformation(self, from_frame, to_frame, matrix, create_if_not_exists=True):
        """Update an existing transformation."""
        if (from_frame, to_frame) in self.transformations or create_if_not_exists:
            self.transformations[(from_frame, to_frame)] = matrix
        elif (to_frame, from_frame) in self.transformations or create_if_not_exists:
            self.transformations[(to_frame, from_frame)] = invert_transform(matrix)
        else:
            if not create_if_not_exists:
                print(f"Transformation from {from_frame} to {to_frame} not found!")

    def print_transformations(self):
        """Print all known transformations."""
        for (from_frame, to_frame), matrix in self.transformations.items():
            print(f"Transformation from {from_frame} to {to_frame}:\n{matrix}\n")

    def print_frames(self):
        """Print all known frames."""
        for frame_name, matrix in self.frames.items():
            print(f"Frame {frame_name}:\n{matrix}\n")
    
    def frames_connected(self, frame_names):
        """Check if all frames are connected."""
        for i in range(len(frame_names)):
            for j in range(i + 1, len(frame_names)):
                transformation = self.get_transformation(frame_names[i], frame_names[j])
                if transformation is None:
                    return False
        return True
    
    def visualize_transformation(self, from_frame, to_frame):
        """Visualize a transformation between two frames."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot the from_frame at origin
        plot_transform(ax=ax, A2B=np.eye(4), name=from_frame, s=0.2)
        
        # Get the transformation matrix
        T = self.get_transformation(from_frame, to_frame)
        if T is None:
            raise ValueError(f"No transformation found between {from_frame} and {to_frame}.")
        
        # Plot the transformed to_frame
        plot_transform(ax=ax, A2B=T, name=to_frame, s=0.2)
        
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")
        
        plt.title(f"Transformation: {from_frame} to {to_frame}")
        plt.show(block=True)
        
    def visualize_transformations(self, frame_pairs, ax=None):
        """Visualize a sequence of transformations between paired frames."""
        # Now working only from pairs' last frame is the next pair's first frame
        # Check the last from of previous pair is the first of the next pair
        for i in range(1, len(frame_pairs)):
            if frame_pairs[i - 1][1] != frame_pairs[i][0]:
                raise ValueError("Frames are not connected.")
        
        if len(frame_pairs) < 1:
            raise ValueError("At least one frame pair is required to visualize transformations.")

        if ax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

        # Initialize current transformation to identity (origin)
        current_transform = np.eye(4)

        # plot the first transfortion from np.eye(4) to the first frame
        plot_transform(ax=ax, A2B=current_transform, name=frame_pairs[0][0], s=0.1)
        
        # Iterate over the list of (from_frame, to_frame) pairs
        for from_frame, to_frame in frame_pairs:
            # check the frame are registered
            if from_frame not in self.frames:
                raise ValueError(f"Frame '{from_frame}' does not exist.")
            
            if to_frame not in self.frames:
                raise ValueError(f"Frame '{to_frame}' does not exist.")
            
            # Get the transformation matrix
            T = self.get_transformation(from_frame, to_frame)
            if T is None:
                raise ValueError(f"No transformation found between {from_frame} and {to_frame}.")

            # Accumulate transformations
            current_transform = np.dot(current_transform, T)

            # Plot the current frame
            plot_transform(ax=ax, A2B=current_transform, name=to_frame, s=0.1)

        # Set plot limits and labels
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")

        # Display the sequence of transformations in the title
        transformation_path = "\n".join([f"{f}->{t}" for f, t in frame_pairs])
        plt.title(f"Transformations: {transformation_path}")
        plt.show(block=True)
        
    # def visualize_transformations_starting_from(self, from_frame, ignore_frames=[]):
    #     """Visualize transformations starting from a given frame."""
    #     # check if the frame exists
    #     if from_frame not in self.frames:
    #         raise ValueError(f"Frame '{from_frame}' does not exist.")
        
    #     # check if the frame is known
    #     if self.frames[from_frame] is None:
    #         raise ValueError(f"Frame '{from_frame}' is unknown.")
        
    #     # Disable interactive mode
    #     plt.ioff()
        
    #     fig = plt.figure()
    #     ax = fig.add_subplot(111, projection='3d')
    #     ax.set_title(f"Transformations Starting from '{from_frame}'")

    #     # Use a set to keep track of visited frames and avoid cycles
    #     visited = set()
    #     self._visualize_transformations_recursively(ax, from_frame, visited, ignore_frames)

    #     # Set plot limits and labels
    #     # Set axis limits and labels by frame values range
    #     known_frames = {frame_name: frame_matrix for frame_name, frame_matrix in self.frames.items() if frame_matrix is not None}
        
    #     ax.set_xlim([-max(np.abs([frame[0, 3] for frame in known_frames.values()])), max(np.abs([frame[0, 3] for frame in known_frames.values()]))])
    #     ax.set_ylim([-max(np.abs([frame[1, 3] for frame in known_frames.values()])), max(np.abs([frame[1, 3] for frame in known_frames.values()]))])
    #     ax.set_zlim([-max(np.abs([frame[2, 3] for frame in known_frames.values()])), max(np.abs([frame[2, 3] for frame in known_frames.values()]))])
        
    #     ax.set_xlabel("X-axis")
    #     ax.set_ylabel("Y-axis")
    #     ax.set_zlabel("Z-axis")

    #     plt.show(block=True)
        
    #     # Re-enable interactive mode
    #     plt.ion()

    # def _visualize_transformations_recursively(self, ax, current_frame, visited, ignore_frames):
    #     """Recursively plot transformations starting from the current frame."""
    #     visited.add(current_frame)

    #      # Plot the current frame if it's not in the ignore list
    #     if current_frame not in ignore_frames and self.frames[current_frame] is not None:
    #         plot_transform(ax=ax, 
    #                        A2B=self.frames[current_frame], 
    #                        name=current_frame, s=0.2)
            

    #     # Traverse all neighboring frames
    #     for neighbor in self._get_to_neighbors(current_frame):
    #         if neighbor in visited:
    #             continue

    #         # Get the transformation matrix from current_frame to neighbor
    #         # if (current_frame, neighbor) in self.transformations:
    #         #     T = self.transformations[(current_frame, neighbor)]
    #         # elif (neighbor, current_frame) in self.transformations:
    #         #     # Use the inverse if the reverse transformation exists
    #         #     T = invert_transform(self.transformations[(neighbor, current_frame)])
    #         # else:
    #         #     raise ValueError(f"No transformation found between '{current_frame}' and '{neighbor}'.")
    #         T = self.get_transformation(current_frame, neighbor)
    #         if T is None:
    #             raise ValueError(f"No transformation found between '{current_frame}' and '{neighbor}'.")
            
    #         # If the neighbor's frame matrix is not known, stop here
    #         if neighbor not in self.frames:
    #             continue
            
    #         if self.frames[current_frame] is not None:
    #             # !!! Do not use @ operator for matrix multiplication, use pytransform3d's concat function !!!
    #             # neighbor_matrix = T @ self.frames[current_frame]
    #             neighbor_matrix = concat(self.frames[current_frame], T)
    #             self.add_frame(neighbor, neighbor_matrix)

    #             # Plot a line between the two frames if not ignored
    #             if current_frame not in ignore_frames and neighbor not in ignore_frames:
    #                 ax.plot(
    #                     [self.frames[current_frame][0, 3], neighbor_matrix[0, 3]],
    #                     [self.frames[current_frame][1, 3], neighbor_matrix[1, 3]],
    #                     [self.frames[current_frame][2, 3], neighbor_matrix[2, 3]],
    #                     'k--'
    #                 )
            
    #         # Recursively plot the next frame
    #         self._visualize_transformations_recursively(ax, neighbor, visited, ignore_frames)
        
        
    # def visualize_known_transformations(self):
    #     """Visualize all known transformations between frames."""
    #     # get all frames with known transformations
    #     frame_names = set()
    #     for (from_frame, to_frame) in self.transformations:
    #         frame_names.add(from_frame)
    #         frame_names.add(to_frame)
            
    #     # visualize transformations between selected frames
    #     self.visualize_transformations(frame_names)
    
            
    # def visualize_transformations(self, frame_names):
    #     """Visualize transformations between selected frames."""
    #     fig = plt.figure()
    #     ax = fig.add_subplot(111, projection='3d')
    #     ax.set_title("Selected Frame Transformations")

    #     # Plot the individual frames
    #     for frame_name in frame_names:
    #         if frame_name in self.frames and self.frames[frame_name] is not None:
    #             plot_transform(ax=ax, A2B=self.frames[frame_name], name=frame_name, s=0.2)

    #     # Plot transformations between the frames
    #     for from_frame in frame_names:
    #         for to_frame in frame_names:
    #             if from_frame != to_frame:
    #                 # Check if transformation exists directly
    #                 if (from_frame, to_frame) in self.transformations:
    #                     T = self.transformations[(from_frame, to_frame)]
    #                 elif (to_frame, from_frame) in self.transformations:
    #                     # Use inverse if the reverse transformation exists
    #                     T = invert_transform(self.transformations[(to_frame, from_frame)])
    #                 else:
    #                     continue  # Skip if no valid transformation exists

    #                 # Plot a line between the two frames if start and end frame are known
    #                 if self.frames[from_frame] is not None and self.frames[to_frame] is not None:
    #                     start = self.frames[from_frame]
    #                     end = concat(start, T)

    #                     ax.plot(
    #                         [start[0, 3], end[0, 3]],
    #                         [start[1, 3], end[1, 3]],
    #                         [start[2, 3], end[2, 3]], 'k--'
    #                     )

    #     # Set axis limits and labels
    #     ax.set_xlim([-2, 5])
    #     ax.set_ylim([-2, 5])
    #     ax.set_zlim([0, 5])
    #     ax.set_xlabel("X-axis")
    #     ax.set_ylabel("Y-axis")
    #     ax.set_zlabel("Z-axis")

    #     plt.show()
                    
                    
        
    # def visualize_all_transformations(self):
    #     """
    #     Visualize all non-None transformations between frames in a 3D plot.
    #     """
    #     fig = plt.figure()
    #     ax = fig.add_subplot(111, projection='3d')
    #     ax.set_title("All Known Transformations")

    #     # Plot all non-None transformations
    #     for (from_frame, to_frame), matrix in self.transformations.items():
    #         start = self.frames.get(from_frame)
    #         end = self.frames.get(to_frame)

    #         # if start is not None and end is not None:
    #         #     # Plot the start and end frames
    #         #     plot_transform(ax=ax, A2B=start, name=from_frame, s=0.1)
    #         #     plot_transform(ax=ax, A2B=end, name=to_frame, s=0.1)

    #         #     # Draw a dashed line between the two frames
    #         #     ax.plot(
    #         #         [start[0, 3], end[0, 3]],
    #         #         [start[1, 3], end[1, 3]],
    #         #         [start[2, 3], end[2, 3]],
    #         #         'k--'
    #         #     )

    #     # Set plot limits and labels
    #     ax.set_xlim([-2, 2])
    #     ax.set_ylim([-2, 2])
    #     ax.set_zlim([0, 2])
    #     ax.set_xlabel("X-axis")
    #     ax.set_ylabel("Y-axis")
    #     ax.set_zlabel("Z-axis")

    #     plt.show()
    
    # def visualize_known_frames(self):
    #     """Visualize all frames and transformations."""
    #     # check if any known frames exist
    #     known_frames = {frame_name: frame_matrix for frame_name, frame_matrix in self.frames.items() if frame_matrix is not None}
    #     if len(known_frames) == 0:
    #         print("No known frames to visualize.")
    #         return
        
    #     fig = plt.figure()
    #     ax = fig.add_subplot(111, projection='3d')
    #     ax.set_title("Frame Transformations")

    #     # Plot all frames
    #     for frame_name, matrix in self.frames.items():
    #         if matrix is not None:
    #             plot_transform(ax=ax, A2B=matrix, name=frame_name, s=0.1)

    #     # Plot all edges between frames
    #     for (from_frame, to_frame), matrix in self.transformations.items():
    #         start = self.frames.get(from_frame)
    #         end = self.frames.get(to_frame)
    #         if start is not None and end is not None:
    #             # Draw a line between the two frames
    #             ax.plot([start[0, 3], end[0, 3]], 
    #                     [start[1, 3], end[1, 3]], 
    #                     [start[2, 3], end[2, 3]], 'k--')

        
    #     # set axis limits and labels by frame values range
    #     ax.set_xlim([min([frame[0, 3] for frame in known_frames.values()]), max([frame[0, 3] for frame in known_frames.values()])])
    #     ax.set_ylim([min([frame[1, 3] for frame in known_frames.values()]), max([frame[1, 3] for frame in known_frames.values()])])
    #     ax.set_zlim([min([frame[2, 3] for frame in known_frames.values()]), max([frame[2, 3] for frame in known_frames.values()])])
        
    #     # set axis names
    #     ax.set_xlabel("X-axis")
    #     ax.set_ylabel("Y-axis")
    #     ax.set_zlabel("Z-axis")

    #     plt.show(block=True)
            
# Example usage
if __name__ == "__main__":
    manager = FrameManager()

    # Example transformations: homogeneous transformation matrices
    T_A_to_B = np.array([[1, 0, 0, 1],
                         [0, 1, 0, 2],
                         [0, 0, 1, 3],
                         [0, 0, 0, 1]])

    T_B_to_C = np.array([[0, -1, 0, 0],
                         [1, 0, 0, 1],
                         [0, 0, 1, 1],
                         [0, 0, 0, 1]])

    manager.add_transformation("A", "B", T_A_to_B)
    manager.add_transformation("B", "C", T_B_to_C)

    # Compute transformation from A to C
    T_A_to_C = manager.compute_transformation("A", "C")
    print("Computed Transformation from A to C:")
    print(T_A_to_C)

    # Update a transformation
    T_A_to_B_new = np.array([[1, 0, 0, 2],
                             [0, 1, 0, 3],
                             [0, 0, 1, 4],
                             [0, 0, 0, 1]])
    manager.update_transformation("A", "B", T_A_to_B_new)

    # Print all transformations
    manager.print_transformations()