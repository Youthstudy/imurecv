# kinematic_model_build_fixed.py
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dataclasses import dataclass
from typing import List
from matplotlib import rcParams
import math
import pandas as pd
from matplotlib.animation import FuncAnimation

rcParams['font.sans-serif'] = ['SimHei']
rcParams['axes.unicode_minus'] = False

@dataclass
class IMUData:
    accelerometer: np.ndarray
    gyroscope: np.ndarray
    quaternion: np.ndarray  # (w, x, y, z)

@dataclass
class EncoderData:
    angle: float  # degrees

@dataclass
class DHParameter:
    a: float
    alpha: float
    d: float
    theta: float  # radians

class LowerLimbKinematicsDH:
    """基于DH参数的下肢运动学模型（支持初始姿态锁定与相对旋转）"""
    def __init__(self, thigh_length=0.45, shank_length=0.43, hip_width=0.2):
        self.thigh_length = thigh_length
        self.shank_length = shank_length
        self.hip_width = hip_width

        # 初始姿态（以 rotation 矩阵形式存储）
        self.init_rotations = {}  # keys: 'pelvis','left_thigh','right_thigh'
        self.initialized = False

        # 内部用于保持角度连续性的历史（弧度）
        self._left_hip_rad_hist: List[float] = []
        self._right_hip_rad_hist: List[float] = []

        # 对外暴露的历史（度, 位置等）
        self.history = {
            'time': [],
            'left_hip_angle': [],    # 度
            'right_hip_angle': [],   # 度
            'left_knee_angle': [],   # 度
            'right_knee_angle': [],  # 度
            'left_joint_positions': [],   # List of lists of 3D points
            'right_joint_positions': []
        }

    # -------------------- 基础数学 / DH --------------------
    def quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        # 输入 q = [w, x, y, z] -> scipy expects [x,y,z,w]
        return R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()

    def dh_transform(self, dh: DHParameter) -> np.ndarray:
        ct = np.cos(dh.theta); st = np.sin(dh.theta)
        ca = np.cos(dh.alpha); sa = np.sin(dh.alpha)
        T = np.array([
            [ct, -st*ca,  st*sa, dh.a*ct],
            [st,  ct*ca, -ct*sa, dh.a*st],
            [0,     sa,     ca,    dh.d],
            [0,     0,      0,     1]
        ])
        return T

    def forward_kinematics_dh(self, pelvis_pose: np.ndarray, dh_params: List[DHParameter]) -> List[np.ndarray]:
        T = pelvis_pose.copy()
        joint_positions = [T[:3, 3].copy()]  # 骨盆位置
        for dh in dh_params:
            T = T @ self.dh_transform(dh)
            joint_positions.append(T[:3, 3].copy())
        return joint_positions

    def get_leg_dh_params(self, hip_angle: float, knee_angle: float, side: str) -> List[DHParameter]:
        hip_offset = self.hip_width / 2.0 if side == 'left' else -self.hip_width / 2.0
        return [
            # pelvis -> hip lateral offset
            DHParameter(a=0.0, alpha=0.0, d=hip_offset, theta=0.0),
            # hip flexion/extension (sagittal plane)
            DHParameter(a=0.0, alpha=0.0, d=0.0, theta=hip_angle),
            # thigh link
            DHParameter(a=self.thigh_length, alpha=0.0, d=0.0, theta=0.0),
            # knee
            DHParameter(a=0.0, alpha=0.0, d=0.0, theta=knee_angle),
            # shank
            DHParameter(a=self.shank_length, alpha=0.0, d=0.0, theta=0.0)
        ]

    # -------------------- 初始姿态与相对旋转 --------------------
    def lock_initial_rotation_if_needed(self, imu_name: str, q_current: np.ndarray):
        """第一次时锁定IMU初始旋转矩阵作为基准R_init"""
        if imu_name not in self.init_rotations:
            self.init_rotations[imu_name] = self.quaternion_to_rotation_matrix(q_current)

    def relative_rotation(self, imu_name: str, q_current: np.ndarray) -> np.ndarray:
        """返回相对于该IMU初始姿态的旋转矩阵 R_rel = R_init^T * R_current"""
        if imu_name not in self.init_rotations:
            # 如果尚未初始化，先锁定
            self.lock_initial_rotation_if_needed(imu_name, q_current)
        R_init = self.init_rotations[imu_name]
        R_curr = self.quaternion_to_rotation_matrix(q_current)
        return R_init.T @ R_curr

    # -------------------- 连续性处理，避免180°跳变 --------------------
    def _ensure_continuous_angle(self, angle_rad: float, hist: List[float]) -> float:
        """利用历史角度保证角度连续性（将 angle_rad 调整到与 hist[-1] 最近的等价值）"""
        if not hist:
            return angle_rad
        prev = hist[-1]
        # 将 angle_rad 移入 prev +/- pi 区间范围
        # 通过添加或减去 2*pi 来消除跳变
        while angle_rad - prev > np.pi:
            angle_rad -= 2 * np.pi
        while angle_rad - prev < -np.pi:
            angle_rad += 2 * np.pi
        return angle_rad

    # -------------------- 关键: 用IMU计算髋关节角 --------------------
    def calculate_hip_angle_from_relative_rot(self, R_pelvis: np.ndarray, R_thigh: np.ndarray) -> float:
        """
        计算髋关节屈曲角度（绕局部Y轴或近似绕前后轴）。
        我们使用相对旋转矩阵 R_rel = R_pelvis^T * R_thigh，然后从其中提取屈曲分量。
        返回值：弧度（未经连续性处理）
        """
        R_rel = R_pelvis.T @ R_thigh
        # 常见方法取绕Y轴 (屈伸)：
        # angle = atan2(R_rel[0,2], R_rel[2,2])
        angle = np.arctan2(R_rel[0, 2], R_rel[2, 2])
        return angle

    # -------------------- 主更新函数 --------------------
    def update(self, timestamp: float,
               pelvis_imu: IMUData,
               left_thigh_imu: IMUData,
               right_thigh_imu: IMUData,
               left_knee_encoder: EncoderData,
               right_knee_encoder: EncoderData,
               pelvis_position: np.ndarray = None):
        """
        更新模型（逐帧调用）
        - 第一次会锁定每个IMU的初始旋转（基准）
        - 后续使用相对旋转计算角度
        """
        if pelvis_position is None:
            pelvis_position = np.array([0.0, 0.0, 0.0])

        # 锁定初始旋转（如果尚未锁定）
        self.lock_initial_rotation_if_needed('pelvis', pelvis_imu.quaternion)
        self.lock_initial_rotation_if_needed('left_thigh', left_thigh_imu.quaternion)
        self.lock_initial_rotation_if_needed('right_thigh', right_thigh_imu.quaternion)
        if not self.initialized:
            # 第一次调用时把初始骨盆姿态设为“竖直向下”参考（仅告知）
            self.initialized = True
            print("✅ 已锁定初始IMU基准姿态（作为 '竖直向下' 起始参考）。")

        # 计算相对于初始的旋转矩阵
        R_pelvis_rel = self.relative_rotation('pelvis', pelvis_imu.quaternion)
        R_left_rel = self.relative_rotation('left_thigh', left_thigh_imu.quaternion)
        R_right_rel = self.relative_rotation('right_thigh', right_thigh_imu.quaternion)

        # 从相对旋转中计算髋角（弧度）
        left_hip_rad = self.calculate_hip_angle_from_relative_rot(R_pelvis_rel, R_left_rel)
        right_hip_rad = self.calculate_hip_angle_from_relative_rot(R_pelvis_rel, R_right_rel)

        # 连续性修正（避免 180° 翻转）
        left_hip_rad = self._ensure_continuous_angle(left_hip_rad, self._left_hip_rad_hist)
        right_hip_rad = self._ensure_continuous_angle(right_hip_rad, self._right_hip_rad_hist)

        # 保存连续性历史（弧度）
        self._left_hip_rad_hist.append(left_hip_rad)
        self._right_hip_rad_hist.append(right_hip_rad)

        # 膝角直接由编码器（度 -> 弧度）
        left_knee_rad = math.radians(left_knee_encoder.angle)
        right_knee_rad = math.radians(right_knee_encoder.angle)

        # 为 DH 前向运动学构造骨盆位姿（将 R_pelvis_rel 用作骨盆朝向）
        # 如果你要把初始定义为“竖直向下”，这里可以再乘以一个固定的旋转矩阵进行对齐，
        # 示例中我们把初始姿态基准的相对旋转直接作为骨盆朝向（在 init 阶段，R_pelvis_rel = I）
        pelvis_pose = np.eye(4)
        # 可选：把坐标轴变换为 Z 向下的世界（示例加了一个对齐变换）
        align_to_z_down = np.array([
            [1, 0, 0],
            [0, 0, 1],
            [0, -1, 0]
        ])
        pelvis_pose[:3, :3] = R_pelvis_rel @ align_to_z_down
        pelvis_pose[:3, 3] = pelvis_position

        # DH 前向
        left_dh = self.get_leg_dh_params(left_hip_rad, left_knee_rad, 'left')
        right_dh = self.get_leg_dh_params(right_hip_rad, right_knee_rad, 'right')
        left_positions = self.forward_kinematics_dh(pelvis_pose, left_dh)
        right_positions = self.forward_kinematics_dh(pelvis_pose, right_dh)

        # 存历史（对外以度表示角度）
        self.history['time'].append(timestamp)
        self.history['left_hip_angle'].append(math.degrees(left_hip_rad))
        self.history['right_hip_angle'].append(math.degrees(right_hip_rad))
        self.history['left_knee_angle'].append(left_knee_encoder.angle)
        self.history['right_knee_angle'].append(right_knee_encoder.angle)
        self.history['left_joint_positions'].append(left_positions)
        self.history['right_joint_positions'].append(right_positions)

        return {
            'left_hip_angle': math.degrees(left_hip_rad),
            'right_hip_angle': math.degrees(right_hip_rad),
            'left_knee_angle': left_knee_encoder.angle,
            'right_knee_angle': right_knee_encoder.angle,
            'left_positions': left_positions,
            'right_positions': right_positions
        }

    # -------------------- 绘图与动画 --------------------
    def plot_joint_angles(self):
        if len(self.history['time']) == 0:
            print("没有数据可以绘制")
            return
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        t = self.history['time']
        axes[0].plot(t, self.history['left_hip_angle'], label='左髋')
        axes[0].plot(t, self.history['right_hip_angle'], label='右髋')
        axes[0].set_ylabel('髋角 (度)')
        axes[0].legend(); axes[0].grid(True); axes[0].axhline(0, color='k', linestyle='--', alpha=0.3)
        axes[1].plot(t, self.history['left_knee_angle'], label='左膝')
        axes[1].plot(t, self.history['right_knee_angle'], label='右膝')
        axes[1].set_ylabel('膝角 (度)'); axes[1].set_xlabel('时间 (s)')
        axes[1].legend(); axes[1].grid(True); axes[1].axhline(0, color='k', linestyle='--', alpha=0.3)
        plt.tight_layout(); plt.show()

    def plot_skeleton_3d(self, frame_idx=-1):
        if len(self.history['time']) == 0:
            print("没有数据")
            return
        left_pos = self.history['left_joint_positions'][frame_idx]
        right_pos = self.history['right_joint_positions'][frame_idx]
        fig = plt.figure(figsize=(10, 8)); ax = fig.add_subplot(111, projection='3d')
        lx = [p[0] for p in left_pos]; ly = [p[1] for p in left_pos]; lz = [p[2] for p in left_pos]
        rx = [p[0] for p in right_pos]; ry = [p[1] for p in right_pos]; rz = [p[2] for p in right_pos]
        ax.plot(lx, ly, lz, '-o', label='左腿'); ax.plot(rx, ry, rz, '-o', label='右腿')
        ax.plot([lx[0], rx[0]],[ly[0], ry[0]],[lz[0], rz[0]], '-g', linewidth=3, label='骨盆')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.invert_zaxis()  # 让 Z 轴向下
        ax.legend(); ax.grid(True)
        plt.title(f'骨架 (帧 {frame_idx})'); plt.show()

    def animate_skeleton(self, interval: int = 20, step: int = 5, repeat=True, save_path: str = None):
        """
        播放动画
        - interval: 毫秒 (务必为正整数)
        """
        if len(self.history['time']) == 0:
            print("没有数据可以播放")
            return None

        if interval <= 0:
            interval = 20

        # 计算全局范围
        all_positions = []
        for L, Rr in zip(self.history['left_joint_positions'], self.history['right_joint_positions']):
            all_positions.extend(L); all_positions.extend(Rr)
        all_x = [p[0] for p in all_positions]; all_y = [p[1] for p in all_positions]; all_z = [p[2] for p in all_positions]
        max_range = max(max(all_x)-min(all_x), max(all_y)-min(all_y), max(all_z)-min(all_z)) / 2.0
        mid_x = (max(all_x) + min(all_x)) * 0.5
        mid_y = (max(all_y) + min(all_y)) * 0.5
        mid_z = (max(all_z) + min(all_z)) * 0.5

        fig = plt.figure(figsize=(14, 10)); ax = fig.add_subplot(111, projection='3d')
        left_line, = ax.plot([], [], [], '-o', linewidth=3, markersize=6, label='左腿')
        right_line, = ax.plot([], [], [], '-o', linewidth=3, markersize=6, label='右腿')
        pelvis_line, = ax.plot([], [], [], '-g', linewidth=4, label='骨盆')
        joints = ['骨盆','髋','大腿','膝','小腿','踝']
        joint_texts = [ax.text(0,0,0,'', fontsize=8) for _ in range(len(joints)*2)]
        time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes)

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.invert_zaxis()
        ax.grid(True); ax.legend(); ax.set_title('下肢步态动画')

        def init():
            left_line.set_data([], []); left_line.set_3d_properties([])
            right_line.set_data([], []); right_line.set_3d_properties([])
            pelvis_line.set_data([], []); pelvis_line.set_3d_properties([])
            time_text.set_text('')
            for t in joint_texts: t.set_text('')
            return [left_line, right_line, pelvis_line, time_text] + joint_texts

        def update(frame_idx):
            L = self.history['left_joint_positions'][frame_idx]
            Rr = self.history['right_joint_positions'][frame_idx]
            lx = [p[0] for p in L]; ly = [p[1] for p in L]; lz = [p[2] for p in L]
            rx = [p[0] for p in Rr]; ry = [p[1] for p in Rr]; rz = [p[2] for p in Rr]
            left_line.set_data(lx, ly); left_line.set_3d_properties(lz)
            right_line.set_data(rx, ry); right_line.set_3d_properties(rz)
            pelvis_line.set_data([lx[0], rx[0]], [ly[0], ry[0]]); pelvis_line.set_3d_properties([lz[0], rz[0]])
            time_text.set_text(f'时间: {self.history["time"][frame_idx]:.2f}s  帧: {frame_idx}/{len(self.history["time"])-1}')
            # joint labels
            ti = 0
            for i, name in enumerate(joints):
                if i < len(L):
                    joint_texts[ti].set_position((lx[i], ly[i])); joint_texts[ti].set_3d_properties(lz[i])
                    joint_texts[ti].set_text(f'L-{name}'); ti += 1
                if i < len(Rr):
                    joint_texts[ti].set_position((rx[i], ry[i])); joint_texts[ti].set_3d_properties(rz[i])
                    joint_texts[ti].set_text(f'R-{name}'); ti += 1
            return [left_line, right_line, pelvis_line, time_text] + joint_texts
        
        anim = FuncAnimation(fig, update, frames=range(0, len(self.history['time']), step), init_func=init,
                             interval=interval, blit=False, repeat=repeat)

        if save_path:
            try:
                print(f"正在保存动画到 {save_path} ...")
                anim.save(save_path, writer='pillow', fps=int(1000/interval))
                print("保存完成。")
            except Exception as e:
                print("保存动画失败:", e)

        plt.show()
        return anim

# -------------------- 辅助函数: 从 CSV 提取 IMU/Encoder --------------------
def extract_imu(df: pd.DataFrame, suffix: str, imu_name: str) -> pd.DataFrame:
    # 尝试选取带 suffix 的列 (例如 "Acc_X_1", "Quat_W_1" 等)
    imu_cols = ['System_Time'] + [col for col in df.columns if col.endswith(suffix)]
    imu_df = df[imu_cols].copy()
    imu_df.columns = ['System_Time'] + [col.replace(suffix, '') for col in imu_df.columns[1:]]
    imu_df['IMU'] = imu_name
    return imu_df

def extract_enc(df: pd.DataFrame, suffix: str, joint_name: str, flag: bool = True) -> pd.DataFrame:
    enc_cols = [col for col in df.columns if col.endswith(suffix)]
    enc_df = df[enc_cols].copy()
    enc_df.columns = [col.replace(suffix, '') for col in enc_df.columns]
    if not flag:
        enc_df = -enc_df
    enc_df['joint'] = joint_name
    return enc_df

def df2encode(enc_row: pd.Series) -> EncoderData:
    # 假设编码器原始值是弧度 ret0
    angle_deg = math.degrees(enc_row['ret0'])
    return EncoderData(angle=angle_deg)

def df2imuData(imu_row: pd.Series) -> IMUData:
    acc = np.array([imu_row.get('Acc_X', 0.0), imu_row.get('Acc_Y', 0.0), imu_row.get('Acc_Z', 0.0)])
    gyro = np.array([imu_row.get('Gyro_X', 0.0), imu_row.get('Gyro_Y', 0.0), imu_row.get('Gyro_Z', 0.0)])
    quat = np.array([imu_row.get('Quat_W', 1.0), imu_row.get('Quat_X', 0.0), imu_row.get('Quat_Y', 0.0), imu_row.get('Quat_Z', 0.0)])
    return IMUData(accelerometer=acc, gyroscope=gyro, quaternion=quat)

# -------------------- 可运行示例主函数 --------------------
if __name__ == "__main__":
    # ===== 修改这里的路径到你的 CSV =====
    DATA_PATH = r'F://3.biye//1_code//imurecv//data//merged//merged_20251106_183706.csv'
    # ===================================
    try:
        df = pd.read_csv(DATA_PATH)
    except Exception as e:
        print("无法读取 CSV，请确认路径。错误：", e)
        raise

    # 创建模型
    model = LowerLimbKinematicsDH(thigh_length=0.50, shank_length=0.49, hip_width=0.37)

    # 根据你原始文件的列后缀来提取 IMU/encoder（示例里使用 _1, _3, _2 等后缀）
    imu0 = extract_imu(df, '_1', 'IMU_pelvis')
    imu1 = extract_imu(df, '_3', 'IMU_left_thigh')
    imu2 = extract_imu(df, '_2', 'IMU_right_thigh')

    enc0 = extract_enc(df, '_joint1', 'joint1', flag=True)
    enc1 = extract_enc(df, '_joint2', 'joint2', flag=False)

    # 检查长度一致性，取最短的那一个作为帧数
    n = min(len(df), len(imu0), len(imu1), len(imu2), len(enc0), len(enc1))
    print(f"读取 {n} 帧数据，采样率假设 fs = 200Hz（如不同请修改代码）")

    fs = 200.0
    t = 0.0
    for i in range(n):
        try:
            pelvis_imu = df2imuData(imu0.iloc[i])
            left_thigh_imu = df2imuData(imu1.iloc[i])
            right_thigh_imu = df2imuData(imu2.iloc[i])
            left_enc = df2encode(enc0.iloc[i])
            right_enc = df2encode(enc1.iloc[i])
        except Exception as e:
            print("行解析错误, 跳过帧:", i, e)
            continue

        t += 1.0/fs
        model.update(timestamp=t,
                     pelvis_imu=pelvis_imu,
                     left_thigh_imu=left_thigh_imu,
                     right_thigh_imu=right_thigh_imu,
                     left_knee_encoder=left_enc,
                     right_knee_encoder=right_enc)

    print("数据更新完成，开始绘图与动画...")
    # 关节角度曲线
    model.plot_joint_angles()

    # 播放动画（每帧 interval 毫秒）
    # 如果动画还是存在翻转问题，请尝试增加 interval（减慢速度）或检查 CSV 的四元数连贯性
    model.animate_skeleton(interval=0.0, repeat=True, save_path=None)

    # 有需要的话也可以显示几个关键帧静态图
    # model.plot_skeleton_3d(frame_idx=0)
    # model.plot_skeleton_3d(frame_idx=len(model.history['time'])//2)
    # model.plot_skeleton_3d(frame_idx=-1)
