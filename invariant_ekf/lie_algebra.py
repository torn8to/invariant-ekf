import numpy as np
from numpy import ndarray
from math import sin, cos
from typing import Callable

lie_tolerance:float = 1e-6
norm:Callable = np.linalg.norm

def skew(phi:np.ndarray)->np.ndarray:
    """
    Args:
        phi (ndarray(shape=(3), dim=1)) : a vector in tangent space of the so3 lie group
    Returns:
        ndarray(dim=2, shape=(3,3)) the skew symmetric matrix of Phi

    """
    skew = np.zeros(9)
    skew[1] = -phi[2]
    skew[2] = phi[1]
    skew[3] = phi[2]
    skew[5] = phi[0]
    skew[6] = -phi[1]
    skew[7] = phi[0]
    return skew.reshape(3,3)

def hat(phi):
    """Alias for skew"""
    return skew(phi)


def exp_so3(v: np.ndarray)-> np.ndarray:
    """ exponential map of the so3 lie group from a tangent this includes a second order taylor series approximation
    Args:
        phi (ndarray(shape=(3), dim=1)) : a vector in tangent space of the so3 lie group
    Returns:
        ndarray(dim=2, shape=(3,3) exponential map of so3 lie group
    """
    theta = norm(v)
    if theta < lie_tolerance:
        return np.identity(3)
    else:
        A = skew(v)
        sin_theta = sin(theta)
        cos_theta = cos(theta)
        return np.identity(3) + sin_theta * A  + .5 * sin_theta * A * A # taylor expansion


def exp_se3(v: np.ndarray, tolerance=lie_tolerance)-> np.ndarray:
    """conver transfomration matrix to its adjoint representation
    Args:
        T (np.ndarray(shape=6,1), dim=2): Trnasformation Matrix

    Returns:
        np.ndarray(shape=(6,6)): adjoint se3 matrix
    """
    identity = np.eye(4)
    T:np.ndarray = np.eye(4)
    rho: np.ndarray = v[:3]
    phi: np.ndarray = v[3:]
    so3_skew = exp_so3(phi)
    theta: float = norm(phi)
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)

    if theta < tolerance:
        J =  identity * 0.5 * so3_skew
    else:
        ax = phi/theta
        K = skew(phi)
        J =  (sin_theta/theta) * np.eye(3)  + (1 - sin_theta/theta) * np.outer(ax, ax) + ((1- cos_theta)/ theta) * K

    t = J @ rho
    T[:3, :3] = so3_skew
    T[:3, 3] = t
    return T


def adjoint_se3(v:np.ndarray):
    """conver transfomration matrix to its adjoint representation
    Args:
        T (np.ndarray(shape=6,1), dim=2): Transformation Matrix

    Returns:
        np.ndarray(shape=(6,6)): adjoint se3 matrix

    """
    upsilon = v[:3]
    omega = v[3:]

    adj = np.zeros((6,6))
    adj[:3, :3]= skew(omega)
    adj[:3,3:] = skew(upsilon)
    adj[3:, 3:]= skew(omega)
    return adj
