import numpy as np


class RobotState:
    def __init__(pose:np.ndarray = np.identiry(4),
                 initial_velocity: np.ndarry = np.zeros(3),
                 angular_velocity: np.ndarray = np.zeros(3),
                  ):
        self.rp = np.identity(4)
        self.v = np.zeros(3)
        self.w = np.zeros(3)

    @property
    def position():
        return self.rp[3,:3]
    
    @property
    def orientation():
        return self.rp[:3,:3]


    def


