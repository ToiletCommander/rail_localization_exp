import numpy as np
from localizer_base import rotation_matrix, rotation_matrix_inverse
for i in range(30):
    for j in range(30):
        for k in range(30):
            rot = rotation_matrix(i,j,k)
            rot_inv = rotation_matrix_inverse(i,j,k)
            assert np.allclose(rot_inv,np.linalg.inv(rot))
