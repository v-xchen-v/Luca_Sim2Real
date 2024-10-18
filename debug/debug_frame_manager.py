import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)

from coordinates.frame_manager import FrameManager

frame_manager = FrameManager()

# Examples of initializing frames
frame_manager.initialize_frames(
    ["real_world", "sim_world", "calibration_board_real", "camera_real"]
)

# Print the initialized frames, with None as the transformation matrix
frame_manager.print_frames()




