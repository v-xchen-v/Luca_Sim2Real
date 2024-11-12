import numpy as np

sim_traj_filepath = f'data/trajectory_data/sim_trajectory/coke_can_1110/step-800.npy'

data = np.load(sim_traj_filepath, allow_pickle=True)

print(f'before fix: {data.item()["right_hand_base_pose_buf"][0]}')
# x:0, y:1, z:2, qw, qx, qy, qz
data.item()['right_hand_base_pose_buf'][:, :, 2] += 0.03
data.item()['right_hand_base_pose_buf'][:, :, 1] -= 0.005
# data.item()['right_hand_base_pose_buf'][:, :, 0] -= 0.015

print(f'after fix: {data.item()["right_hand_base_pose_buf"][0]}')
np.save(f'data/trajectory_data/sim_trajectory/coke_can_1110/step-800_finetuned.npy',
        data, allow_pickle=True)