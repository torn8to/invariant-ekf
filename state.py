from dataclasses import dataclass, field
from scipy.spatial.transform import Rotation
import numpy as np
import torch
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

@dataclass(slots=True)
class ImuUpdate:
    timetamp: float
    linear_acceleration: np.ndarray
    angular_velocity : np.ndarray
    orientation: np.ndarray

    @classmethod
    def fromMsg(cls, msg: Imu):
        pass

@dataclass(slots=True)
class OdomUpdate:
    timetamp: float
    position: np.ndarray
    quaternion: np.ndarray
    twist: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float64))
    pose_covariance: np.ndarray = field(default_factory=lambda: np.zeros(36, dtype=np.float64).reshape(6,6))
    twist_covariance: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64).reshape(6,6))

    def __post_init__(self):
        pass

    @classmethod
    def fromMsg(cls, msg: Odometry):
        pass


@dataclass(slots=True)
class RobotState:
    _orientation: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float64) )
    _position: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    _velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    _acceleration_bias: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    _angular_velocity_bias: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))


    def __post_init__(self):
        try:
            self._orientation = Rotation.from_quat(self._orientation).as_quat() # normalize quaternion
        except:
            self._orientation[3] = 1.0
            self._orientation = Rotation.from_quat(self._orientation).as_quat() # normalize quaternion


        assert self._orientation.shape == (4,)
        assert self._position.shape == (3,)
        assert self._velocity.shape == (3,)
        assert self._acceleration_bias.shape == (3,)
        assert self._angular_velocity_bias.shape == (3,)


    def convertToPyPose(self) -> list[torch.Tensor]:
        pass
