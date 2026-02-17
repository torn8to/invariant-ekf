import numpy as np
from math import sin, cos

lie_tolerance:float = 1e-6

def skew(v:np.ndarray)->np.ndarray:
    skew = np.zeros(9)
    skew[1] = -v[2]
    skew[2] = v[1]
    skew[3] = v[2]
    skew[5] = v[0]
    skew[6] = -v[1]
    skew[7] = v[0]
    return skew.reshape(3,3)


def exp_so3(v: np.ndarray)-> np.ndarray:
    theta = v.norm()
    if theta < lie_tolerance:
        return np.identity(3)
    else:
        A = skew(v)
        sin_theta = sin(theta)
        return np.identity(3) + sin_theta * A  + .5 * sin_theta * A * A # taylor expansion


def exp_se3(v: np.ndarray)-> np.ndarray:
    se3_mat:np.ndarray = np.eye(4)
    rot_v = v[0:3]
    so3_skew = exp_so3(v)
    # TODO implent exponential map


def adjoint_se3():
    # TODO implement
    pass

    
