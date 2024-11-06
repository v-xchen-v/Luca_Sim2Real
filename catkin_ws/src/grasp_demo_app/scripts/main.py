from core.grasp_and_place_executor import GraspAndPlaceExecutor
from core.executable_trajectory_generator import ExecutableTrajectoryGenerator


def setup_scene():
    print("Setting up scene") # place holder for setup scene at very first then no need to repeat
    pass

def generate_executable_trajectory(traj_generator: ExecutableTrajectoryGenerator):
    traj_generator.generate_trajectory()

def execuate_trajectory():
    pass

def main():
    # Initialize objects
    traj_generator = ExecutableTrajectoryGenerator()    
    grasp_and_place_executor = GraspAndPlaceExecutor()
    
    repeat_times = 10
    setup_scene()
    
    for i in range(repeat_times):
        print(f"\n--- Iteration {i+1} ---")
        
        # Moveto home position
        grasp_and_place_executor.goto_home()
        
        # Generate and execute trajectory
        generate_executable_trajectory(traj_generator)
        grasp_and_place_executor.grasp('trace.npy')
        
        # Return to home position
        grasp_and_place_executor.goto_home()
        
        # Pause to allow the user to press Enter to continue
        input("Press Enter to continue...")
        
        
if __name__ == '__main__':
    main()