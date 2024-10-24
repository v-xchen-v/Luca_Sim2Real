import numpy as np

VIS_HAND_IN_ROBOT_COORDINATE = True


traj_name='coka_can_1017'
save_path = f"data/trajectory_data/real_trajectory/{traj_name}/step-0.npy"

# Step 6: Reload and check the shape and content of data
traj_real_data = np.load(save_path)
print(traj_real_data.shape)

[traj_real_data[:7]]
## Visualize the transformation between robot_right_hand_base in robot_base in real world
if VIS_HAND_IN_ROBOT_COORDINATE:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from pytransform3d.transformations import plot_transform
    import matplotlib
    matplotlib.use('TkAgg')  # Or 'Qt5Agg', depending on your setup
    # Set up the plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")



    def update_frame(i):
        ax.cla()  # Clear the current frame
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        # add a text
        
        transformation = T_right_hand_base_steps_to_robot_base_real[i]
        # plot_transform(ax, A2B=transformation, s=0.1)  # s sets the size of the frame
        adaptor.frame_manager.update_transformation("robot_base_real", "right_hand_base_real", transformation)
        # show the dynamic transformation of the object, right_hand_base_real, with respect to readable_real frame 
        # and optional reference robot_base_real frame and camera_real frame
        adaptor.frame_manager.visualize_transformations(
            [
                ("robot_base_real", "right_hand_base_real"),
                ("right_hand_base_real", "object_real"),
                ("object_real", "readable_real"),
            ], 
            ax, block=False)

    # Create animation
    anim = FuncAnimation(fig, update_frame, frames=int(len(T_right_hand_base_steps_to_robot_base_real)/5), interval=100)

    # Display the animation
    plt.show(block=True)