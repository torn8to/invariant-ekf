import numpy as np
from state import RobotState, ImuReading
from filterpy.kalman import ExtendedKalmanFilter
from scipy.spatial
from dataclasses import dataclass

gravity_vec: np.ndarray = np.array([0, 0 , -9.81])

@dataclass
class PreintegrationResult:
    orientation: np.ndarray
    position: np.ndarray
    velocity: np.ndarray



class InertialIEKF():
    gravity_vec: np.ndarray = np.array([0, 0 , -9.81])

    def __init__(self, dim_x=15, dim_z=6):
        pass



    def preintegrate_imu(state: RobotState, imu_update: ImuReading)-> np.ndarray:
        last_time = state.time
        position: np.ndarray = state.position
        orientation: np.ndarray = state.orientation
        velocity: np.ndarray = state.velocity_delta
        current_time: float =  imu_update.current_time
        measured_acceleration = imu_update.acceleration
        measured_angular_velocity = imu_update.angular_velocity
        acceleration_bias: np.ndarray = state.accel_bias
        angular_velocity_bias: np.ndarray =  state.angular_velocity_bias


        delta_time: float = current_time - last_time
        processed_acceleration: np.ndarray =  measured_acceleration - linear_acceleration.bias
        gravity_removed_acceleration: np.ndarray  = processed_acceleration - (np.linalg.inv(orientation) @ gravity_vec)

        print(gravity_removed_acceleration)
        linear_velocity_delta: np.ndarray = (gravity_removed_acceleration * delta_time)/2
        angular_delta: float =  (angular_velocity * delta_time)/2
        half_angle: np.ndarray = angular_delta/2
        half_angle_mapped: np.ndarray = exp_so3(half_angle)
        orientation_half_update = orientation @ half_angle_mapped
        # velocity * orientation + position 
        position_post_velocity_update:np.ndarray = orientation_half_update @ velocity + position
        orientation_full_update = orientation_half_update @ position
        return PreintegrationStepResult(orientation_full_update,
                                        position_post_velocity_update,
                                        updated_velocity)

    def predict(imu_update: ImuReading):
        preintegration_result:PreintegrationStepResult = self.preintegrate_imu(self.state, imu_update)


    def update(np.ndarray):
        pass




