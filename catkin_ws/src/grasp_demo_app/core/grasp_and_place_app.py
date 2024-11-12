import os, sys


# Add module path for importing custom modules
module_path = os.path.abspath(os.path.join('catkin_ws/src/grasp_demo_app'))
if module_path not in sys.path:
    sys.path.append(module_path)

from core.grasp_and_place_executor import GraspAndPlaceExecutor
from core.executable_trajectory_generator import ExecutableTrajectoryGenerator
from pointcloud_processing.pointcloud_exceptions import ICPFitnessException
import json
import rospy

RIGHT_ARM_URDF_PATH = 'catkin_ws/src/MSRA_SRobot_core/src/robot_arm_pkg/assets/Realman_Inspire_R/Realman_Inspire_R.urdf'

class GraspAndPlaceApp:
    def __init__(self, config_path):
        """Initialize the application with the necessary configurations."""
        self.traj_generator = ExecutableTrajectoryGenerator(sim2real_traj_config=config_path)
        self.executor = GraspAndPlaceExecutor(RIGHT_ARM_URDF_PATH)
        
        # load configs
        self._load_config(config_path)
        self.execution_enabled=self.config["execution_enabled"]
        
        ## visualize config
        self.vis_sim_init = self.config["vis_sim_init"]
        self.vis_sim_traj = self.config["vis_sim_traj"]
        self.vis_object_in_real = self.config["vis_object_in_real"]
        self.vis_real_hand_approach_object = self.config["vis_real_hand_approach_object"]
        
        self.pregrasp_eef_pose = None
        self.pregrasp_hand_angles = None

    # Step 1: Setup
    def setup_environment(self):
        """Set up the scene at the beginning."""
        print("Setting up scene")
        self.traj_generator.initialize()
        # calibration board on the center of table margin near to robot
        notouch_table_dimensions = [1.0, 1.2, 0.055+0.1]
        # notouch_table_dimensions = [0.1, 0.1, 0.1]
        self.table_obstacle = self.traj_generator.processor.real_traj_adaptor.get_restricted_table_no_touch_zone_in_robot_coord(
            notouch_table_dimensions
        )
        print(f"table_obstacle: {self.table_obstacle}")

    # Step 2: Prepare Trajectory
    def prepare_trajectory(self, vis_pregrasp_pose=False):
        """Generate the trajectory using the provided trajectory generator."""
        self.traj_generator.generate_trajectory(
            vis_sim_initial_setup=self.vis_sim_init,
            anim_sim_hand_approach=self.vis_sim_traj,
            vis_object_in_real=self.vis_object_in_real,
            anim_real_hand_approach_object=self.vis_real_hand_approach_object,
        )
        
        # Compute pregrasp pose by generated grasping trajectory
        self.pregrasp_eef_pose, self.pregrasp_hand_angles = \
            self._get_pregrasp_pose(t_scale=self.traj_generator.object_manager_configs["pregrasp_tscale"], 
                                    vis=vis_pregrasp_pose)

    def execute(self):
        """Execute grasp and place, including moving to pregrasp position, executing trajectory, and placing object."""
        # Move to pregrasp position
        # TODO: config this vis switch to file
        
        self._move_to_pregrasp_position()
        self._execute_trajectory(self.traj_generator.traj_file_path, 
                                steps=self.traj_generator.object_manager_configs["first_n_steps"], 
                                hz=self.traj_generator.object_manager_configs["grasp_traj_hz"],
                                hand_offset_at_n_step=self.traj_generator.object_manager_configs["hand_offset_at_n_step"],
                                hand_offset=self.traj_generator.object_manager_configs["hand_offset"],
                                hand_preoffset_for_all_steps=self.traj_generator.object_manager_configs["hand_preoffset_for_all_steps"])
        self.executor.lift(0.1)
        self.executor.goto_preplace(
                                table_obstacle=self.table_obstacle)
        self.executor.open_hand()
        self.executor.goto_home(table_obstacle=self.table_obstacle)
        
    def _execute_trajectory(self, trajectory_file, steps=120, hz=2, 
                            hand_offset_at_n_step=50, hand_offset=3,
                            hand_preoffset_for_all_steps=0):
        """Execute the generated trajectory with the specified parameters."""
        self.executor.grasp(trajectory_file, first_n_steps=steps, hz=hz,
                            hand_offset_at_n_step=hand_offset_at_n_step, hand_offset=hand_offset,
                            hand_preoffset_for_all_steps=hand_preoffset_for_all_steps)

    def _get_pregrasp_pose(self, t_scale, vis):
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
        if vis:
            self.traj_generator.processor.real_traj_adaptor.visualize_tscale_hand_to_object_at_step0(t_scale)
        return eef_pose, finger_angles

    def _move_to_pregrasp_position(self):
        """Move the executor to the pregrasp position.
        Parameters:
            t_scale (float): The scaling factor for the pregrasp pose, 
            which is used to adjust the relative position of end effector with respect to the object.
            hz (int): The frequency at which the robot should move.
        """

        self.executor.goto_pregrasp(pregrasp_eef_pose_matrix=self.pregrasp_eef_pose, 
                                    pregrasp_hand_angles=self.pregrasp_hand_angles, 
                                    table_obstacle=self.table_obstacle)

    def _grasp_and_place_cycle(self, repeat_count=10, allow_moveit_failure=True):
        """Run the grasp and place process multiple times.
        allow errors to occur and continue next iteration:
        1. plan failed
        2. icp fitness too low
        """
        for iteration in range(repeat_count):
            print(f"\n--- Iteration {iteration + 1} ---")
            if iteration == 0:
                if self.execution_enabled:
                    self.executor.goto_home(table_obstacle=self.table_obstacle)

            
            # handle errors that can occur during the process and goto next iteration
            try:
                self.prepare_trajectory()
            except ICPFitnessException as e:
                print(f"Error in prepare_trajectory: {e}")
                continue
            
            if self.execution_enabled:
                if allow_moveit_failure:
                    try:
                        self.execute()
                    except rospy.ServiceException as e:
                        print(f"Can not reach target position with moveit planner: {e}")
                        print(f"Moveit planning failed, or service not available.")
                        input("Press Enter to continue next grasp iteration...")
                        continue
                else:
                    self.execute()
            
            input("Press Enter to continue next grasp iteration...")

    def run(self, repeat_count=10):
        """Setup the table robot camera calibration."""
        self.setup_environment()
        input("Environment ready, Press Enter to continue...")
        
        """Execute the full grasp and place cycle."""
        self._grasp_and_place_cycle(repeat_count)

    def _load_config(self, config):
        # Load configuration from a dictionary or a json file
        if isinstance(config, str):
            with open(config, 'r') as f:
                self.config = json.load(f)
        elif isinstance(config, dict):
            self.config = config
        else:
            raise ValueError("Invalid config type. Use a dictionary or a json file.")
def main():
    # Configuration for the system
    sim2real_config = 'catkin_ws/src/grasp_demo_app/config/debug_cam_calibration_config_11f_coke.json'
    app = GraspAndPlaceApp(sim2real_config)
    app.run(repeat_count=10)


if __name__ == '__main__':
    main()
