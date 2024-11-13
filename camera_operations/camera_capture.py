import pyrealsense2 as rs
import numpy as np
import cv2
import os

class RealSenseCamera:
    #TODO: throw error when initialize failed
    def __init__(self):
        """Initialize the Intel RealSense camera pipeline."""
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Enable the RGB stream
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # Start the camera pipeline
        self.profile = self.pipeline.start(self.config)
        


    def get_rgb_frame(self):
        """
        Retrieve a single RGB frame from the camera.

        Returns:
        - color_image: The captured RGB frame as a NumPy array.
        """
        # Wait for the next set of frames from the camera
        frames = self.pipeline.wait_for_frames()

        # Extract the color frame
        color_frame = frames.get_color_frame()

        if not color_frame:
            raise RuntimeError("Failed to retrieve RGB frame")

        # Convert the frame to a NumPy array
        color_image = np.asanyarray(color_frame.get_data())

        return color_image

    def save_image(self, color_image, save_path="images/captured_frame.jpg"):
        """
        Save an RGB frame to the specified path.

        Parameters:
        - color_image: The RGB frame to save as a NumPy array.
        - save_path: The path where the image will be saved.
        """
        # Create the directory if it doesn't exist
        os.makedirs(os.path.dirname(save_path), exist_ok=True)

        # Save the image
        cv2.imwrite(save_path, color_image)
        print(f"Image saved at {save_path}")

    def release(self):
        """Stop the pipeline and release resources."""
        self.pipeline.stop()

    def run_loop(self):
        """
        Continuously capture and display frames.
        Press 's' to save an image and 'q' to quit.
        """
        print("Starting camera loop. Press 's' to save an image or 'q' to quit.")

        while True:
            try:
                # Get the RGB frame
                frame = self.get_rgb_frame()

                # Display the frame in an OpenCV window
                cv2.imshow("RGB Frame", frame)

                # Wait for a key press
                key = cv2.waitKey(1) & 0xFF

                if key == ord('s'):
                    # Save the current frame if 's' is pressed
                    self.save_image(frame, save_path="images/captured_frame.jpg")
                elif key == ord('q'):
                    # Exit the loop if 'q' is pressed
                    print("Quitting the loop.")
                    break

            except RuntimeError as e:
                print(e)
                break

        # Release resources
        self.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    camera = RealSenseCamera()
    camera.run_loop()