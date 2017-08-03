from pybonirob import transformations
import math
import numpy as np


def test_mat_quat():
    alpha, beta, gamma = 0.0, 0.0, math.pi / 2
    xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)

    # Rotation matrices
    Rx = transformations.rotation_matrix(alpha, xaxis)
    Ry = transformations.rotation_matrix(beta, yaxis)
    Rz = transformations.rotation_matrix(gamma, zaxis)
    R = transformations.concatenate_matrices(Rx, Ry, Rz)

    qx = transformations.quaternion_about_axis(alpha, xaxis)
    qy = transformations.quaternion_about_axis(beta, yaxis)
    qz = transformations.quaternion_about_axis(gamma, zaxis)
    q = transformations.quaternion_multiply(qx, qy)
    q = transformations.quaternion_multiply(q, qz)
    Rq = transformations.quaternion_matrix(q)

    assert transformations.is_same_transform(R, Rq)


def test_quat_rpy():
    alpha, beta, gamma = math.pi / 4, math.pi / 3, math.pi / 2

    q = transformations.quaternion_from_euler(alpha, beta, gamma, 'sxyz')
    roll, pitch, yaw = transformations.euler_from_quaternion(q, 'sxyz')

    assert np.allclose([roll, pitch, yaw], [alpha, beta, gamma])
