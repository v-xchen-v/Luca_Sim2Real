import os, sys

# Add module path for importing custom modules
module_path = os.path.abspath(os.path.join('catkin_ws/src/grasp_demo_app'))
if module_path not in sys.path:
    sys.path.append(module_path)

from core.grasp_and_place_executor import GraspAndPlaceExecutor
from core.executable_trajectory_generator import ExecutableTrajectoryGenerator


class GraspAndPlaceApp:
    def __init__(self, config_path):
        """Initialize the application with the necessary configurations."""
        self.traj_generator = ExecutableTrajectoryGenerator(sim2real_traj_config=config_path)
        self.executor = GraspAndPlaceExecutor()
        self.setup_environment()

    def setup_environment(self):
        """Set up the scene at the beginning."""
        print("Setting up scene")
        self.traj_generator.initialize()

    def generate_trajectory(self):
        """Generate the trajectory using the provided trajectory generator."""
        self.traj_generator.generate_trajectory()

    def execute_trajectory(self, trajectory_file, steps=120, hz=2):
        """Execute the generated trajectory with the specified parameters."""
        self.executor.grasp(trajectory_file, first_n_steps=steps, hz=hz)

    def get_pregrasp_pose(self, t_scale=1):
        """Generate and return the pregrasp pose and finger angles."""
        eef_pose = self.traj_generator.processor.get_tscaled_robotbase_to_hand_at_first_point(t_scale)
        finger_angles = self.traj_generator.processor.get_finger_angles_at_first_point()
        return eef_pose, finger_angles

    def move_to_pregrasp_position(self, t_scale=1, hz=2):
        """Move the executor to the pregrasp position."""
        eef_pose, finger_angles = self.get_pregrasp_pose(t_scale)
        self.executor.goto_pregrasp(pregrasp_eef_pose_matrix=eef_pose, pregrasp_hand_angles=finger_angles, hz=hz)

    def grasp_and_place_cycle(self, repeat_count=10):
        """Run the grasp and place process multiple times."""
        for iteration in range(repeat_count):
            print(f"\n--- Iteration {iteration + 1} ---")
            self.executor.goto_home()
            self.generate_trajectory()
            self.move_to_pregrasp_position(t_scale=2, hz=2)
            self.execute_trajectory(self.traj_generator.traj_file_path, steps=120, hz=2)
            self.executor.lift(0.1)
            self.executor.goto_preplace()
            self.executor.open_hand()
            self.executor.goto_home()
            input("Press Enter to continue...")

    def run(self, repeat_count=10):
        """Execute the full grasp and place cycle."""
        self.grasp_and_place_cycle(repeat_count)


def main():
    # Configuration for the system
    sim2real_config = 'catkin_ws/src/grasp_demo_app/config/debug_cam_calibration_config_11f_coke.json'
    app = GraspAndPlaceApp(sim2real_config)
    app.run(repeat_count=10)


if __name__ == '__main__':
    main()
