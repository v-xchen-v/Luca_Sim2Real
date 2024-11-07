"""Grasp orange multiple times"""

"""
- conda activate py38
- source /opt/ros/noetic/setup.bash
- source ./catkin_ws/devel/setup.bash
- then, run this script
"""
import os, sys
module_path = os.path.abspath(os.path.join('catkin_ws/src/grasp_demo_app'))
if module_path not in sys.path:
    sys.path.append(module_path)


from core.grasp_and_place_executor import GraspAndPlaceExecutor
from core.executable_trajectory_generator import ExecutableTrajectoryGenerator

def setup_scene():
    print("Setting up scene") # place holder for setup scene at very first then no need to repeat
    pass

def generate_executable_trajectory(traj_generator: ExecutableTrajectoryGenerator):
    traj_generator.generate_trajectory()

def execuate_trajectory(grasp_and_place_executor: GraspAndPlaceExecutor, trajectory_file, first_n_steps, hz):
    grasp_and_place_executor.grasp(trajectory_file, 
                                   first_n_steps=first_n_steps,
                                   hz=hz)

def generate_pregrasp_pose(traj_generator: ExecutableTrajectoryGenerator, t_scale=1):
    tscaled_eef_pose_at_first_point = traj_generator.processor.get_tscaled_robotbase_to_hand_at_first_point(t_scale)
    finger_angles_at_first_point = traj_generator.processor.get_finger_angles_at_first_point()
    return tscaled_eef_pose_at_first_point, finger_angles_at_first_point
    
    
def pregrasp(traj_generator, grasp_and_place_executor, t_scale=1, hz=2):
    tscaled_eef_pose_at_first_point, finger_angles_at_first_point = generate_pregrasp_pose(traj_generator=traj_generator, t_scale=t_scale)
    grasp_and_place_executor.goto_pregrasp(pregrasp_eef_pose_matrix=tscaled_eef_pose_at_first_point, 
                                           pregrasp_hand_angles=finger_angles_at_first_point, 
                                           hz=hz)
    
def main():
    # Config
    sim2real_config = 'catkin_ws/src/grasp_demo_app/config/debug_cam_calibration_config_11f_coke.json'
    
    # Initialize objects
    traj_generator = ExecutableTrajectoryGenerator(sim2real_traj_config=sim2real_config)    
    grasp_and_place_executor = GraspAndPlaceExecutor()
    
    repeat_times = 10
    setup_scene()
    
    for i in range(repeat_times):
        print(f"\n--- Iteration {i+1} ---")
        
        # Moveto home position
        grasp_and_place_executor.goto_home()
        
        # Generate traj and pregrasp position
        generate_executable_trajectory(traj_generator)
        
        
        # Pregrasp
        pregrasp(traj_generator, grasp_and_place_executor, t_scale=1, hz=2)
        
        # Generate and execute trajectory
        execuate_trajectory(grasp_and_place_executor, traj_generator.traj_file_path, first_n_steps=120, hz=2)
        
        # Place object
        grasp_and_place_executor.lift(0.1)
        grasp_and_place_executor.goto_preplace()
        grasp_and_place_executor.open_hand()
        
        # Return to home position
        grasp_and_place_executor.goto_home()
        
        # Pause to allow the user to press Enter to continue
        input("Press Enter to continue...")
        
        
if __name__ == '__main__':
    main()