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


def setup_environment():
    """Set up the scene at the beginning."""
    print("Setting up scene")
    # place holder for setup scene at very first then no need to repeat
    pass

def generate_trajectory(traj_generator: ExecutableTrajectoryGenerator):
    """Generate the trajectory using the provided trajectory generator."""
    traj_generator.generate_trajectory()

def execute_trajectory(executor: GraspAndPlaceExecutor, trajectory_file, steps=120, hz=2):
    """Execute the generated trajectory with the specified parameters."""
    executor.grasp(trajectory_file, first_n_steps=steps, hz=hz)


def get_pregrasp_pose(traj_generator: ExecutableTrajectoryGenerator, t_scale=1):
    """Generate and return the pregrasp pose and finger angles."""
    eef_pose = traj_generator.processor.get_tscaled_robotbase_to_hand_at_first_point(t_scale)
    finger_angles = traj_generator.processor.get_finger_angles_at_first_point()
    return eef_pose, finger_angles

def move_to_pregrasp_position(traj_generator: ExecutableTrajectoryGenerator, executor: GraspAndPlaceExecutor, t_scale=1, hz=2):
    """Move the executor to the pregrasp position."""
    eef_pose, finger_angles = get_pregrasp_pose(traj_generator, t_scale)
    executor.goto_pregrasp(pregrasp_eef_pose_matrix=eef_pose, pregrasp_hand_angles=finger_angles, hz=hz)
    
def grasp_and_place_cycle(traj_generator: ExecutableTrajectoryGenerator, executor: GraspAndPlaceExecutor, repeat_count=10):
    """Run the grasp and place process multiple times."""
    for iteration in range(repeat_count):
        print(f"\n--- Iteration {iteration + 1} ---")
        
        # Move to home position
        executor.goto_home()
        
        # Generate trajectory and pregrasp position
        generate_trajectory(traj_generator)
        move_to_pregrasp_position(traj_generator, executor, t_scale=2, hz=2)
        
        # Execute grasp trajectory
        execute_trajectory(executor, traj_generator.traj_file_path, steps=120, hz=2)
        
        # Place the object
        executor.lift(0.1)
        executor.goto_preplace()
        executor.open_hand()
        
        # Return to home position
        executor.goto_home()
        
        # Wait for user input to proceed
        input("Press Enter to continue...")
     
        
def main():
    # Config
    sim2real_config = 'catkin_ws/src/grasp_demo_app/config/debug_cam_calibration_config_11f_coke.json'
    
    # Initialize the trajectory generator and executor
    traj_generator = ExecutableTrajectoryGenerator(sim2real_traj_config=sim2real_config)    
    traj_generator.initialize()
    grasp_and_place_executor = GraspAndPlaceExecutor()
    
    # Set up the environment once
    setup_environment()
    
    # Execute the grasp and place cycle
    grasp_and_place_cycle(traj_generator, grasp_and_place_executor, repeat_count=10)
        
if __name__ == '__main__':
    main()