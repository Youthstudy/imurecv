import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dataclasses import dataclass
from typing import Tuple, List
from matplotlib import rcParams

from bt_connect import IMUSensorData


# 设置中文字体
rcParams['font.sans-serif'] = ['SimHei']
rcParams['axes.unicode_minus'] = False


@dataclass
class EncoderData:
    angle: float

@dataclass
class DHParameter:
    a: float
    alpha: float
    d: float
    theta: float

class LowerLimbKinematicsDH:
    def __init__(self, thigh_length=0.45, shank_length=0.43, hip_width=0.2):
        self.thigh_length = thigh_length
        self.shank_length = shank_length
        self.hip_width = hip_width
        
        self.history = {
            'time': [],
            'left_hip_angle': [],
            'right_hip_angle': [],
            'left_knee_angle': [],
            'right_knee_angle': [],
            'left_joint_positions': [],
            'right_joint_positions': []
        }
    
    def dh_transform(self, dh: DHParameter) -> np.ndarray:
        ct, st = np.cos(dh.theta), np.sin(dh.theta)
        ca, sa = np.cos(dh.alpha), np.sin(dh.alpha)
        return np.array([
            [ct, -st*ca,  st*sa, dh.a*ct],
            [st,  ct*ca, -ct*sa, dh.a*st],
            [0,   sa,     ca,    dh.d],
            [0,   0,      0,     1]
        ])
    
    def quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        return R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    
    def get_leg_dh_params(self, hip_angle: float, knee_angle: float, side: str) -> List[DHParameter]:
        hip_offset = self.hip_width / 2 if side == 'left' else -self.hip_width / 2
        return [
            DHParameter(0, 0, hip_offset, 0),
            DHParameter(0, 0, 0, hip_angle),
            DHParameter(self.thigh_length, 0, 0, 0),
            DHParameter(0, 0, 0, knee_angle),
            DHParameter(self.shank_length, 0, 0, 0)
        ]
    
    def forward_kinematics_dh(self, pelvis_pose: np.ndarray, dh_params: List[DHParameter]) -> List[np.ndarray]:
        T = pelvis_pose.copy()
        joint_positions = [T[:3, 3]]
        for dh in dh_params:
            T = T @ self.dh_transform(dh)
            joint_positions.append(T[:3, 3].copy())
        return joint_positions
    
    def calculate_hip_angle_from_imu(self, pelvis_imu: IMUSensorData, thigh_imu: IMUSensorData) -> float:
        R_pelvis = self.quaternion_to_rotation_matrix(pelvis_imu.quaternion)
        R_thigh = self.quaternion_to_rotation_matrix(thigh_imu.quaternion)
        R_relative = R_pelvis.T @ R_thigh
        return np.arctan2(R_relative[0, 2], R_relative[2, 2])
    
    def update(self, timestamp: float,
               pelvis_imu: IMUSensorData,
               left_thigh_imu: IMUSensorData,
               right_thigh_imu: IMUSensorData,
               left_knee_encoder: EncoderData,
               right_knee_encoder: EncoderData,
               pelvis_position: np.ndarray = None):
        if pelvis_position is None:
            pelvis_position = np.array([0.0, 0.0, 0.0])
        
        R_pelvis = self.quaternion_to_rotation_matrix(pelvis_imu.quaternion)
        pelvis_pose = np.eye(4)
        pelvis_pose[:3, :3] = R_pelvis
        pelvis_pose[:3, 3] = pelvis_position
        
        left_hip_angle = self.calculate_hip_angle_from_imu(pelvis_imu, left_thigh_imu)
        right_hip_angle = self.calculate_hip_angle_from_imu(pelvis_imu, right_thigh_imu)
        
        left_knee_angle = np.radians(left_knee_encoder.angle)
        right_knee_angle = np.radians(right_knee_encoder.angle)
        
        left_dh = self.get_leg_dh_params(left_hip_angle, left_knee_angle, 'left')
        right_dh = self.get_leg_dh_params(right_hip_angle, right_knee_angle, 'right')
        
        left_positions = self.forward_kinematics_dh(pelvis_pose, left_dh)
        right_positions = self.forward_kinematics_dh(pelvis_pose, right_dh)
        
        self.history['time'].append(timestamp)
        self.history['left_hip_angle'].append(np.degrees(left_hip_angle))
        self.history['right_hip_angle'].append(np.degrees(right_hip_angle))
        self.history['left_knee_angle'].append(left_knee_encoder.angle)
        self.history['right_knee_angle'].append(right_knee_encoder.angle)
        self.history['left_joint_positions'].append(left_positions)
        self.history['right_joint_positions'].append(right_positions)
        
        return {
            'left_hip_angle': np.degrees(left_hip_angle),
            'right_hip_angle': np.degrees(right_hip_angle),
            'left_knee_angle': left_knee_encoder.angle,
            'right_knee_angle': right_knee_encoder.angle,
            'left_positions': left_positions,
            'right_positions': right_positions
        }
    
    def plot_skeleton_3d(self, frame_idx=-1):
        if len(self.history['time']) == 0:
            print("没有数据可以绘制")
            return
        
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        left_pos = self.history['left_joint_positions'][frame_idx]
        right_pos = self.history['right_joint_positions'][frame_idx]
        
        left_x = [p[0] for p in left_pos]
        left_y = [p[1] for p in left_pos]
        left_z = [p[2] for p in left_pos]
        right_x = [p[0] for p in right_pos]
        right_y = [p[1] for p in right_pos]
        right_z = [p[2] for p in right_pos]
        
        ax.plot(left_x, left_y, left_z, 'b-o', linewidth=3, label='左腿')
        ax.plot(right_x, right_y, right_z, 'r-o', linewidth=3, label='右腿')
        ax.plot([left_x[0], right_x[0]], [left_y[0], right_y[0]], [left_z[0], right_z[0]], 'g-', linewidth=4, label='骨盆')
        
        ax.set_xlabel('X (米)')
        ax.set_ylabel('Y (米)')
        ax.set_zlabel('Z (米)')
        ax.set_title(f'下肢骨架 (DH参数法) - 时间: {self.history["time"][frame_idx]:.2f}s')
        ax.legend()
        plt.show()
    
    def plot_joint_angles(self):
        if len(self.history['time']) == 0:
            print("没有数据可以绘制")
            return
        
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        time = self.history['time']
        
        axes[0].plot(time, self.history['left_hip_angle'], 'b-', label='左髋')
        axes[0].plot(time, self.history['right_hip_angle'], 'r-', label='右髋')
        axes[0].set_ylabel('髋关节角度 (度)')
        axes[0].legend()
        axes[0].grid(True)
        
        axes[1].plot(time, self.history['left_knee_angle'], 'b-', label='左膝')
        axes[1].plot(time, self.history['right_knee_angle'], 'r-', label='右膝')
        axes[1].set_xlabel('时间 (秒)')
        axes[1].set_ylabel('膝关节角度 (度)')
        axes[1].legend()
        axes[1].grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def print_dh_table(self, hip_angle: float, knee_angle: float, side: str):
        dh_params = self.get_leg_dh_params(np.radians(hip_angle), np.radians(knee_angle), side)
        print(f"\n{side.upper()}腿DH参数表:")
        print("-" * 60)
        print(f"{'关节':<12}{'a(m)':<12}{'α(rad)':<12}{'d(m)':<12}{'θ(rad)':<12}")
        print("-" * 60)
        names = ['骨盆-髋', '髋关节', '大腿', '膝关节', '小腿']
        for n, d in zip(names, dh_params):
            print(f"{n:<12}{d.a:<12.4f}{d.alpha:<12.4f}{d.d:<12.4f}{d.theta:<12.4f}")
        print("-" * 60)


# 示例运行
if __name__ == "__main__":
    model = LowerLimbKinematicsDH(shank_length=0.43, thigh_length=0.45, hip_width=0.2)

    dt, duration = 0.01, 2.0
    timestamps = np.arange(0, duration, dt)
    
    model.print_dh_table(30, 45, 'left')
    print("\n模拟步态数据...")
    for t in timestamps:
        pelvis_imu = IMUSensorData(np.array([0, 9.81, 0]), np.zeros(3), np.array([1, 0, 0, 0]))
        left_imu = IMUSensorData(np.array([0, 9.81, 0]), np.zeros(3), np.array([1, 0, 0, 0]))
        right_imu = IMUSensorData(np.array([0, 9.81, 0]), np.zeros(3), np.array([1, 0, 0, 0]))
        left_encoder = EncoderData(angle=20 * np.sin(2*np.pi*t))
        right_encoder = EncoderData(angle=-20 * np.sin(2*np.pi*t))
        model.update(t, pelvis_imu, left_imu, right_imu, left_encoder, right_encoder, np.array([0, 0, 1]))
    
    model.plot_joint_angles()
    model.plot_skeleton_3d()
