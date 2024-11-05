from core.grasp_and_place_executor import GraspAndPlaceExecutor

def setup_scene():
    pass

def generate_executable_trajectory():
    pass

def execuate_trajectory():
    pass

def main():
    
    grasp_and_place_executor = GraspAndPlaceExecutor()
    
    repeat_times = 10
    setup_scene()
    
    for i in range(repeat_times):
        
        grasp_and_place_executor.goto_home()
        
        generate_executable_trajectory()
        
        grasp_and_place_executor.grasp('trace.npy')
        
        grasp_and_place_executor.goto_home()
        
        # press key to continue, for dummy