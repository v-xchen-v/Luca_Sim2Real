import os, sys
# module_path = os.path.abspath(os.path.join('.'))
module_path = r'MSRA_SRobot_core/src/robot_arm_pkg/scripts/robot_arm_py'
if module_path not in sys.path:
    sys.path.append(module_path)
    
from robot_arm_py.arm_robot_control import ArmRobots

robot = ArmRobots(
    right_arm_ip="192.168.10.18",
    right_ee_type="hand",
)

robot.on_arm_ee_direct_command("")