# imu_adapter.py
import numpy as np
from dataclasses import dataclass
from .bt_connect import IMUSensorData
from base.data_base import IMUData

def sensor_to_imu(sensor: IMUSensorData) -> IMUData:
    """
    Adapter:
    bt_connect.IMUSensorData -> kinematic.IMUData
    """
    return IMUData(
        accelerometer=np.array(
            [sensor.acc_x, sensor.acc_y, sensor.acc_z], dtype=float
        ),
        gyroscope=np.array(
            [sensor.gyro_x, sensor.gyro_y, sensor.gyro_z], dtype=float
        ),
        quaternion=np.array(
            [sensor.quat_w, sensor.quat_x, sensor.quat_y, sensor.quat_z],
            dtype=float
        )
    )
