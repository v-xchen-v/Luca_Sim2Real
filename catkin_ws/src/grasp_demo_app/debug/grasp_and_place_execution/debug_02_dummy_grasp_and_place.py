"""Better way to approac the pregrasp"""

import os, sys
module_path = os.path.abspath(os.path.join('catkin_ws/src/grasp_demo_app'))
if module_path not in sys.path:
    sys.path.append(module_path)

# The simplest pipe: home->grasp traj -> home

from core.grasp_and_place_executor import GraspAndPlaceExecutor

grasp_executor = GraspAndPlaceExecutor()
grasp_executor.goto_home()

traj_filepath = 'catkin_ws/src/grasp_demo_app/debug/grasp_and_place_execution/data/real_trajectory/sunscreen_1101/step-0.npy'
grasp_executor.goto_pregrasp(traj_filepath) # Necessary? If need slower speed when approaching?
grasp_executor.grasp(traj_filepath)

grasp_executor.goto_preplace()
grasp_executor.open_hand()

grasp_executor.goto_home()