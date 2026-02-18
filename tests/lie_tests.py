import unittest
import random
from numpy import np
from lie_algebra import skew, exp_so3, exp_se3, adj_se3
import sophuspy as sp
np.random.seed(42)


def sophus_is_close(sophus, arr) -> bool:
    return np.allclose(sophus.matrix(), arr)

def generate_so3_tangent_space_vectors(n:int):
    return (np.random(n,3) - 0.5) * 2

def generate_se3_tangent_space_vectors(n:int):
    data = np.zeros(n, 6)
    omega = generate_so3_tangent_space_vectors(n)
    upsilons = (np.random(n,3) - 0.5) * 10
    data[:, :3] =  upsilons
    data[:, 3:] =  omega
    return data

class FuzzyExponentialMapLieGroupTests(unittest.TestCase):
    num_fuzz_cases:int = 1000
    def fuzz_so3_exponential_map():
        test_vectors: np.ndarray = generate_so3_tangent_space_vectors(num_fuzz_cases)
        result_buffer: np.ndarray = np.zeros((num_fuzz_cases,3,3))
        sophus_buffer: list[sp.SO3] = []
        for i in range(num_fuzz_cases):
            result_buffer[i] = exp_so3(test_vector[i])
            sophus_buffer.append(sp.SO3.exp(test_vector[i]))
        for i in range(num_fuzz_cases):
            assertTrue(sophus_is_close(tf_sophus_list[i], result_matrix[i]))

    def fuzz_se3_exponential_map():
        test_vectors: np.ndarray = generate_se3_tangent_space_vectors(num_fuzz_cases)
        result_buffer: np.ndarray = np.zeros((num_fuzz_cases,3,3))
        sophus_buffer: list[sp.SE3] = []
        for i in range(num_fuzz_cases):
            result_buffer[i] = exp_se3(test_vector[i])
            sophus_buffer.append(sp.SE3.exp(test_vector[i]))
        for i in range(num_fuzz_cases):
            assertTrue(sophus_is_close(tf_sophus_list[i], result_matrix[i]))


class FuzzyAdjointSE3Tests(unitest.TestCase):
    def fuzz_adjoint_SE3():
        test_vectors: np.ndarray = generate_se3_tangent_space_vectors(num_fuzz_cases)
        result_buffer: np.ndarray = np.zeros((num_fuzz_cases, 6, 6))
        sophus_buffer: np.ndarray = np.zeros(num_fuzz_cases, 6, 6))

        def assemble_adjoint(v:np.ndarray) -> np.ndarray:
            adjoint_mat = np.zeros(6,6)
            se3: sp.SE3 = sp.SE3.exp(v)
            R = se3.rotationMatrix()
            t = se3.translation()
            adjoint_mat[:3, :3] = R
            adjoint_mat[:3, 3:] = sp.so3.hat(t) * R
            adjoint_mat[3:, 3:] = R
            return adjoint_mat
        for i in range(num_fuzz_cases):
            result_buffer[i] = adjoint_se3(test_vector[i])
            sophus_buffer[i] = assemble_adjoint(test_vector[i]))
        for i in range(num_fuzz_cases):
            assertTrue(sophus_is_close(tf_sophus_list[i], result_matrix[i]))
        


