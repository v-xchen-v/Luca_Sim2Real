import os, sys

# Add module path for importing custom modules
module_path = os.path.abspath(os.path.join('catkin_ws/src/grasp_demo_app'))
if module_path not in sys.path:
    sys.path.append(module_path)


from core.grasp_and_place_app import GraspAndPlaceApp

def main():
    # Configuration for the system
    sim2real_config = 'catkin_ws/src/grasp_demo_app/config/debug_cam_calibration_config_11f_orange.json'
    app = GraspAndPlaceApp(sim2real_config)
    app.run(repeat_count=10)
    
main()