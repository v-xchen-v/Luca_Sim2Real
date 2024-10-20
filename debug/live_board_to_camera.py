import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    
from camera_operations.camera_capture import RealSenseCamera
from calibration.calibration_precomputed_data_loader import load_camera_intrinsics_from_npy
from calibration.calibrate_board_to_camera import compute_table_to_camera
from calibration.calibration_exceptions import CalibrationBoardNotFoundError, ReprojectionThresholdExceededError
from calibration.visualize_table_to_camera import visualize_table_to_camera
from matplotlib import pyplot as plt
from pytransform3d.transformations import plot_transform
import numpy as np
import cv2
from coordinates.transformation_utils import create_transformation_matrix
from scipy.spatial.transform import Rotation as R

pattern_size = (5, 8)  # Checkerboard size (rows, columns)
square_size = 0.03  # Size of a square on the checkerboard (in meters)
error_threshold = 0.5
report_dir = "data/debug_data/table_calibration/output/cam"
intrinsics_dir = "calibration/calibration_data/camera1/camera_intrinsics"

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


            # Load the camera intrinsic matrix and distortion coefficients
            mtx, dist = load_camera_intrinsics_from_npy(intrinsics_dir)
            

    
            # Display the frame in an OpenCV window
            cv2.imshow("RGB Frame", frame)

            # Wait for a key press
            key = cv2.waitKey(1) & 0xFF

            if key == ord('s'):
                # Save the current frame if 's' is pressed
                camera.save_image(frame, save_path="images/captured_frame.jpg")
            elif key == ord('q'):
                # Exit the loop if 'q' is pressed
                print("Quitting the loop.")
                break
            
            # Compute the transformation matrix from the calibration board to the camera
            try:
                T_table_to_camera = compute_table_to_camera(frame, pattern_size, square_size, mtx, dist, report_dir=None, error_threshold=None)

                # Visualize the transformation dynamically
                    # Clear the previous plot
                ax.cla()
                ax.set_xlim([-1, 1])
                ax.set_ylim([-1, 1])
                ax.set_zlim([-1, 1])
                ax.set_xlabel("X-axis")
                ax.set_ylabel("Y-axis")
                ax.set_zlabel("Z-axis")

                new_transform = T_table_to_camera # np.linalg.inv(T_table_to_camera)
                # Re-plot the initial frame and the new dynamic frame
                
                # calibration_board_frame, x to right, y to down, z to back, 4x4 transformation matrix
                # construct a rotation matrix, 
                # T_world_to_calibration_board_local = create_transformation_matrix([0, 0, 0], R.from_rotvec([np.pi, 0, np.pi/2]).as_matrix())
                
                # Assum the calibration board is at the origin of the world frame
                world_real_frame = np.array([[1, 0, 0, 0],
                                             [0, 1, 0, 0],
                                            [0, 0, 1, 0],
                                            [0, 0, 0, 1]])
                
                calibration_board_frame = np.array([[0, -1, 0, 0],
                                                    [-1, 0, 0, 0],
                                                    [0, 0, -1, 0],
                                                    [0, 0, 0, 1]])
                
                from coordinates.transformation_utils import create_relative_transformation
                T_real_world_to_calibration_board = create_relative_transformation(calibration_board_frame, world_real_frame)
                # calibration_board_frame = np.array([[1, 0, 0, 0],
                #                                     [0, -1, 0, 0],
                #                                     [0, 0, -1, 0],
                #                                     [0, 0, 0, 1]])
                camera_frame = T_table_to_camera@T_real_world_to_calibration_board@world_real_frame
                plot_transform(ax=ax, A2B=world_real_frame, name="world_real")  # Frame at origin
                # plot_transform(ax=ax, A2B=calibration_board_frame, name="Start Frame")  # Frame at origin
                plot_transform(ax=ax, A2B=camera_frame, name="camera_real")  # New frame

                # Pause to create the animation effect
                plt.pause(0.1)
                
            except CalibrationBoardNotFoundError as e:
                print(e)
                continue
            except ReprojectionThresholdExceededError as e:
                print(e)
                continue


        except RuntimeError as e:
            print(e)
            break

    # Release resources
    camera.release()
    cv2.destroyAllWindows()