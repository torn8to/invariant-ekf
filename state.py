import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from dataclasses import dataclass
from lie_algebra import  skew, hat, exp_so3, exp_se3


@dataclass
class ImuReading:
    timestamp: float
    accelerometer: np.ndarray
    angular_velocity: np.ndarray
    orientation: np.ndarray

    @classmethod
    def fromMsg(cls, msg: Imu):
        stamp: float = msg.header.stamp 
        accel:np.ndarray = np.array([msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z])
        angular_velocity: np.ndarray = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z])
        orienation: np.ndarray =  np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w])
        return cls(stamp, accel, angular_velocity, orientation)
        



class RobotState:
    def __init__(pose:np.ndarray = np.identiry(4),
                 initial_velocity: np.ndarry = np.zeros(3),
                 angular_velocity: np.ndarray = np.zeros(3),
                 accelerometer_bias:np.ndarray = np.zeros(3),
                 angular_velocity_bias:np.ndarray = np.zeros(3)):
        self.rp = pose
        self.v = initial_velocity
        self.accelerometer_bias = accelerometer_bias
        self.angular_velocity_bias = angular_velocity_bias

    @property
    def position(self):
        return self.rp[:3,3]
    
    @property
    def orientation(self):
        return self.rp[:3,:3]

    def orientation_log(self):


