from dataclasses import dataclass
import numpy as np

@dataclass
class IMUData:
    accelerometer: np.ndarray
    gyroscope: np.ndarray
    quaternion: np.ndarray
    
    def to_array(self) -> np.ndarray:
        return np.concatenate([self.accelerometer, self.gyroscope, self.quaternion])


@dataclass
class EncoderData:
    angle: float
    velocity: float = 0.0
    t_ff: float = 0.0
    
    def to_array(self) -> np.ndarray:
        return np.array([self.angle, self.velocity, self.t_ff])
    
@dataclass
class DHParameter:
    a: float
    alpha: float
    d: float
    theta: float
    def to_array(self) -> np.ndarray:
        return np.array([self.a, self.alpha, self.d, self.theta])


    
