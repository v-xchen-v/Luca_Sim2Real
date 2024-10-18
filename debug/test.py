import numpy as np
from scipy.spatial.transform import Rotation as R

# rotation_matrix = R.from_rotvec([np.pi, 0, -np.pi]).as_matrix()
# print(rotation_matrix)

rotation_matrix = np.array([[1, 0, 0],
                            [0, -1, 0],
                            [0, 0, -1]])
print(R.from_matrix(rotation_matrix).as_rotvec())