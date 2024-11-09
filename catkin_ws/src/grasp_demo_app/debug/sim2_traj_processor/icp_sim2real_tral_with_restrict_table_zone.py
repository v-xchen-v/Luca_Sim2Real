import os, sys
module_path = os.path.abspath(os.path.join('catkin_ws/src/grasp_demo_app'))
if module_path not in sys.path:
    sys.path.append(module_path)

from core.sim2real_trajectory_processor import Sim2RealTrajectoryProcessor

# Example usage
processor = Sim2RealTrajectoryProcessor(config='catkin_ws/src/grasp_demo_app/config/debug_cam_calibration_config_11f_coke.json')
table_width = 0.6
table_height = 0.6 * 2
object_height = 0.8 +0.3
table_thickness = 0.3 + object_height
processor.setup_robot_table(table_dimensions=[table_width, table_height, table_thickness])
print("Robot table setup completed.")
print(processor.restricted_table_no_touch_zone)