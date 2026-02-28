import numpy as np
from dataclasses import dataclass, field


@dataclass(slots=True)
class NoiseParams:
    _gyroCov: np.ndarray = field(default_factory=lambda: np.identity(3))
    _accelCov: np.ndarray = field(default_factory=lambda: np.identity(3))
    _gyroBiasNoise: np.ndarray = field(default_factory=lambda: np.identity(3))
    _accelBiasNoise: np.ndarray = field(default_factory=lambda: np.identity(3))
    #_lidarNoise: np.ndarray = field(default_factory = lambda: np.ndarry)

    def __post_init__(self):
        assert self._gyroCov.shape == (3, 3)
        assert self._accelCov.shape == (3, 3)
        assert self._gyroBiasNoise.shape == (3, 3)
        assert self._accelBiasNoise.shape == (3, 3)

    @property
    def gyroscopeCovariance(self):
        return self._gyroCov

    @gyroscopeCovariance.setter
    def gyroscopeCovariance(self, val: np.ndarray | float):
        if isinstance(val, float):
            self._gyroCov = np.identity(3) * val
        elif isinstance(val, np.ndarray):
            self._gyroCov = val
        else:
            raise TypeError("not a numpy array or of python type float")

    @property
    def accelerometerCovariance(self):
        return self._gyroCov

    @accelerometerCovariance.setter
    def gyroscopeCovariance(self, val: np.ndarray | float):
        if isinstance(val, float):
            self._accelCov = np.identity(3) * val
        elif isinstance(val, np.ndarray):
            self._accelCov = val
        else:
            raise TypeError("not a numpy array or of python type float")

    @property
    def gyroBiasNoise(self):
        return self._gyroBiasNoise

    @gyroBiasNoise.setter
    def gyroBiasNoise(self, val: np.ndarray | float):
        if isinstance(val, float):
            self._gyroBiasNoise = np.identity(3) * val
        elif isinstance(val, np.ndarray):
            self._gyroBiasNoise = val
        else:
            raise TypeError("not a numpy array or of python type float")

    @property
    def accelBiasNoise(self):
        return self._accelBiasNoise

    @accelBiasNoise.setter
    def gyroBiasNoise(self, val: np.ndarray | float):
        if isinstance(val, float):
            self._accelBiasNoise = np.identity(3) * val
        elif isinstance(val, np.ndarray):
            self._accelBiasNoise = val
        else:
            raise TypeError("not a numpy array or of python type float")


if __name__ == "__main__":
    NoiseParams()
