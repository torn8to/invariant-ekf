import numpy as np
from state import RobotState, ImuUpdate, OdomUpdate
from filterpy.kalman import ExtendedKalmanFilter
from lie_algebra import exp_so3, exp_se3, hat 
from dataclasses import dataclass

gravity_vec: np.ndarray = np.array([0, 0, -9.81])

@dataclass(slots=True)
class PreintegrationStepResult:
    orientation: np.ndarray
    position: np.ndarray
    velocity: np.ndarray


class InertialIEKF():
    gravity_vec: np.ndarray = np.array([0, 0 , -9.81])

    def __init__(self, state: RobotState | None = None, dim_x=12, dim_z=6):
        self._state = state if state is not None else RobotState()

    @property
    def state(self):
        return self._state

    @property
    def postion(self):
        pass

    def preintegrate_imu(state: RobotState, imu_update: ImuUpdate)-> np.ndarray:
        # Local import to keep module importable even if lie_algebra is under development.
        last_time = state.time
        position: np.ndarray = state.position
        orientation: np.ndarray = state.orientation
        velocity: np.ndarray = state.velocity_delta
        current_time: float =  imu_update.current_time

        measured_acceleration = imu_update.acceleration
        measured_angular_velocity = imu_update.angular_velocity

        acceleration_bias: np.ndarray = state.accel_bias
        angular_velocity_bias: np.ndarray =  state.angular_velocity_bias

        angular_velocity = measured_angular_velocity - angular_velocity_bias
        delta_time: float = current_time - last_time
        processed_acceleration: np.ndarray =  measured_acceleration - acceleraton_bias
        gravity_removed_acceleration: np.ndarray  = processed_acceleration - (np.linalg.inv(orientation) @ gravity_vec)
        print(gravity_removed_acceleration)

        linear_velocity_delta: np.ndarray = (gravity_removed_acceleration * delta_time)/2
        mid_velocity = velocity + linear_velocity_delta/2

        angular_delta: float = (angular_velocity * delta_time)/2
        half_angle: np.ndarray = angular_delta/2
        half_angle_mapped: np.ndarray = exp_so3(half_angle)

        # building
        orientation_half_update = orientation @ half_angle_mapped
        position_post_velocity_update:np.ndarray = orientation_half_update @ mid_velocity + position
        orientation_full_update = orientation_half_update @ position

        full_velocity_update = velocity + linear_velocity_delta
        return PreintegrationStepResult(orientation_full_update,
                                        position_post_velocity_update,
                                        full_velocity_update)

    def predict(imu: ImuUpdate):
        preintegration_result: PreintegrationStepResult = self.preintegrate_imu(self.state, imu)

        # udpate robot state

    def update(odom: OdomUpdate):
        pass
