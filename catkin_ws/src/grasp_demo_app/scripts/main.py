import os, sys

# Add module path for importing custom modules
module_path = os.path.abspath(os.path.join('catkin_ws/src/grasp_demo_app'))
if module_path not in sys.path:
    sys.path.append(module_path)


from core.grasp_and_place_app import GraspAndPlaceApp

def main():
    # Configuration for the systems
    sim2real_config = 'catkin_ws/src/grasp_demo_app/config/debug_config_2f.json'
    app = GraspAndPlaceApp(sim2real_config)
    app.run(repeat_count=1000)

main()