import os, sys

# Add module path for importing custom modules
module_path = os.path.abspath(os.path.join('catkin_ws/src/grasp_demo_app'))
if module_path not in sys.path:
    sys.path.append(module_path)

from core.grasp_and_place_executor import GraspAndPlaceExecutor
from core.executable_trajectory_generator import ExecutableTrajectoryGenerator

RIGHT_ARM_URDF_PATH = 'catkin_ws/src/MSRA_SRobot_core/src/robot_arm_pkg/assets/Realman_Inspire_R/Realman_Inspire_R.urdf'

class GraspAndPlaceApp:
    def __init__(self, config_path):
        """Initialize the application with the necessary configurations."""
        self.traj_generator = ExecutableTrajectoryGenerator(sim2real_traj_config=config_path)
        self.executor = GraspAndPlaceExecutor(RIGHT_ARM_URDF_PATH)

    def setup_environment(self):
        """Set up the scene at the beginning."""
        print("Setting up scene")
        self.traj_generator.initialize()
        notouch_table_dimensions = [1.0, 0.8, 0.055+0.1]
        self.table_obstacle = self.traj_generator.processor.real_traj_adaptor.get_restricted_table_no_touch_zone_in_robot_coord(
            notouch_table_dimensions
        )

    def generate_trajectory(self):
        """Generate the trajectory using the provided trajectory generator."""
        self.traj_generator.generate_trajectory()

    def execute_trajectory(self, trajectory_file, steps=120, hz=2):
        """Execute the generated trajectory with the specified parameters."""
        self.executor.grasp(trajectory_file, first_n_steps=steps, hz=hz)

    def _get_pregrasp_pose(self, t_scale=1, vis=False):
        """Generate and return the pregrasp pose and finger angles.
        
        Parameters:
            t_scale (float): The scaling factor for the pregrasp pose, 
            which is used to adjust the relative position of end effector with respect to the object.
            
        Returns:
            tuple: The pregrasp pose(4x4 matrix) and finger angles.
        """
        eef_pose = self.traj_generator.processor.get_tscaled_robotbase_to_hand_at_first_point(t_scale)
        finger_angles = self.traj_generator.processor.get_finger_angles_at_first_point()
        #TODO: move this vis switch to config file
        if False:
            self.traj_generator.processor.real_traj_adaptor.visualize_tscale_hand_to_object_at_step0(t_scale)
        return eef_pose, finger_angles

    def move_to_pregrasp_position(self, t_scale=1, hz=1, vis=False, type='direct', direct_if_moveit_failed=False):
        """Move the executor to the pregrasp position.
        Parameters:
            t_scale (float): The scaling factor for the pregrasp pose, 
            which is used to adjust the relative position of end effector with respect to the object.
            hz (int): The frequency at which the robot should move.
        """
        

        eef_pose, finger_angles = self._get_pregrasp_pose(t_scale, vis)
        self.executor.goto_pregrasp(pregrasp_eef_pose_matrix=eef_pose, pregrasp_hand_angles=finger_angles, hz=hz,
                                    table_obstacle=self.table_obstacle)

    def grasp_and_place_cycle(self, repeat_count=10):
        """Run the grasp and place process multiple times."""
        for iteration in range(repeat_count):
            print(f"\n--- Iteration {iteration + 1} ---")
            self.executor.goto_home(type="moveit",
                                    table_obstacle=self.table_obstacle)
            
            self.generate_trajectory()
            
            # Move to pregrasp position
            # TODO: config this vis switch to file
            self.move_to_pregrasp_position(t_scale=1.2, hz=2, vis=False)
            self.execute_trajectory(self.traj_generator.traj_file_path, 
                                    steps=self.traj_generator.object_manager_configs["first_n_steps"], 
                                    hz=self.traj_generator.object_manager_configs["grasp_traj_hz"])
            self.executor.lift(0.1)
            self.executor.goto_preplace(type="moveit",
                                    table_obstacle=self.table_obstacle)
            self.executor.open_hand()
            self.executor.goto_home()
            input("Press Enter to continue...")

    def run(self, repeat_count=10):
        """Setup the table robot camera calibration."""
        self.setup_environment()
        
        input("Press Enter to continue...")
        
        """Execute the full grasp and place cycle."""
        self.grasp_and_place_cycle(repeat_count)


def main():
    # Configuration for the system
    sim2real_config = 'catkin_ws/src/grasp_demo_app/config/debug_cam_calibration_config_11f_coke.json'
    app = GraspAndPlaceApp(sim2real_config)
    app.run(repeat_count=10)


if __name__ == '__main__':
    main()
