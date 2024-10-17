import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    
from camera_operations.camera_capture import RealSenseCamera

if __name__ == "__main__":
    # Create an instance of the RealSenseCamera class
    camera = RealSenseCamera()

    # Start the camera loop
    camera.run_loop()