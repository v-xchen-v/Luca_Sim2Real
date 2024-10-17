# live show checkerboard corners

import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    
from camera_operations.camera_capture import RealSenseCamera
from calibration.calibration_data_loader import load_camera_intrinsic_from_npy
from calibration.calibrate_board_to_camera import compute_table_to_camera
from calibration.calibration_exceptions import CalibrationBoardNotFoundError, ReprojectionThresholdExceededError
from calibration.visualize_table_to_camera import visualize_table_to_camera
from matplotlib import pyplot as plt
from pytransform3d.transformations import plot_transform
import numpy as np
import cv2
from coordinates.transformations import create_transformation_matrix
from scipy.spatial.transform import Rotation as R
from calibration.checkerboard_utils import detect_checkerboard_corners, draw_detected_corners

pattern_size = (5, 8)  # Checkerboard size (rows, columns)
square_size = 0.03  # Size of a square on the checkerboard (in meters)

if __name__ == "__main__":
    # Create an instance of the RealSenseCamera class
    camera = RealSenseCamera()

    # Initialize the 3D figure and axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([-3, 3])
    # Add reference frames (optional)
    plot_transform(ax=ax, A2B=np.eye(4), name="Start Frame")  # Frame at origin



    """
    Continuously capture and display frames.
    Press 's' to save an image and 'q' to quit.
    """
    print("Starting camera loop. Press 's' to save an image or 'q' to quit.")

    while True:
        try:
            # Get the RGB frame
            frame = camera.get_rgb_frame()

            # detect corners and draw
            found, corners = detect_checkerboard_corners(frame, pattern_size)
            
            frame_draw = draw_detected_corners(frame, corners, pattern_size, found)
    
            # Display the frame in an OpenCV window
            if frame_draw is None:
                cv2.imshow("RGB Frame", frame)
            else:
                cv2.imshow("RGB Frame", frame_draw)

            # Wait for a key press
            key = cv2.waitKey(1) & 0xFF

            if key == ord('s'):
                # Save the current frame if 's' is pressed
                camera.save_image(frame, save_path="images/captured_frame.jpg")
            elif key == ord('q'):
                # Exit the loop if 'q' is pressed
                print("Quitting the loop.")
                break
            
        except RuntimeError as e:
            print(e)
            break

    # Release resources
    camera.release()
    cv2.destroyAllWindows()