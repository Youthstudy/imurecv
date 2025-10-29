import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dataclasses import dataclass
from typing import Tuple, List
from matplotlib import rcParams

import bt_conect

# 设置中文字体（SimHei 是黑体，Linux 可用 Noto Sans CJK）
rcParams['font.sans-serif'] = ['SimHei']  # 或者 ['Microsoft YaHei']
rcParams['axes.unicode_minus'] = False     # 解决负号 '-' 显示成方块的问题

@dataclass
class IMUData:
    """IMU传感器数据结构"""
    accelerometer: np.ndarray  # 3D加速度 (m/s^2)
    gyroscope: np.ndarray      # 3D角速度 (rad/s)
    quaternion: np.ndarray     # 四元数 (w, x, y, z)
    
@dataclass
class EncoderData:
    """编码器数据结构"""
    angle: float  # 膝关节角度 (度)

@dataclass
class DHParameter:
    """DH参数"""
    a: float      # 连杆长度
    alpha: float  # 连杆扭角
    d: float      # 连杆偏距
    theta: float  # 关节角度

class LowerLimbKinematicsDH:
    """基于DH参数的下肢运动学模型"""
    
    def __init__(self, thigh_length=0.45, shank_length=0.43, hip_width=0.2):
        """
        初始化运动学模型
        
        参数:
            thigh_length: 大腿长度 (米)
            shank_length: 小腿长度 (米)
            hip_width: 髋关节宽度 (米)
        """
        self.thigh_length = thigh_length
        self.shank_length = shank_length
        self.hip_width = hip_width
        
        # 存储历史数据
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
        """
        根据DH参数计算变换矩阵
        
        DH变换矩阵 = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
        
        参数:
            dh: DH参数
        返回:
            4x4齐次变换矩阵
        """
        ct = np.cos(dh.theta)
        st = np.sin(dh.theta)
        ca = np.cos(dh.alpha)
        sa = np.sin(dh.alpha)
        
        T = np.array([
            [ct, -st*ca,  st*sa, dh.a*ct],
            [st,  ct*ca, -ct*sa, dh.a*st],
            [0,   sa,     ca,    dh.d],
            [0,   0,      0,     1]
        ])
        
        return T
    
    def quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """将四元数转换为旋转矩阵"""
        return R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    
    def get_leg_dh_params(self, hip_angle: float, knee_angle: float, side: str) -> List[DHParameter]:
        """
        获取单腿的DH参数链
        
        坐标系定义（改进的DH约定）:
        - 基座（骨盆）-> 髋关节 -> 膝关节 -> 踝关节
        
        参数:
            hip_angle: 髋关节角度（弧度）- 屈曲为正
            knee_angle: 膝关节角度（弧度）- 弯曲为正
            side: 'left' 或 'right'
        返回:
            DH参数列表
        """
        # 髋关节偏移方向
        hip_offset = self.hip_width / 2 if side == 'left' else -self.hip_width / 2
        
        dh_params = [
            # 关节0: 骨盆到髋关节（平移）
            DHParameter(
                a=0,           # 无连杆长度
                alpha=0,       # 无扭转
                d=hip_offset,  # 髋关节横向偏移
                theta=0        # 无旋转
            ),
            # 关节1: 髋关节（矢状面屈曲/伸展）
            DHParameter(
                a=0,              # 无横向偏移
                alpha=0,   
                d=0,              # 无纵向偏移
                theta=hip_angle   # 髋关节角度
            ),
            # 关节2: 大腿段
            DHParameter(
                a=self.thigh_length,  # 大腿长度
                alpha=0,              # 平行
                d=0,
                theta=0
            ),
            # 关节3: 膝关节
            DHParameter(
                a=0,
                alpha=0,
                d=0,
                theta=knee_angle  # 膝关节角度
            ),
            # 关节4: 小腿段
            DHParameter(
                a=self.shank_length,  # 小腿长度
                alpha=0,
                d=0,
                theta=0
            )
        ]
        
        return dh_params
    
    def forward_kinematics_dh(self, pelvis_pose: np.ndarray, 
                              dh_params: List[DHParameter]) -> List[np.ndarray]:
        """
        使用DH参数进行前向运动学计算
        
        参数:
            pelvis_pose: 4x4骨盆位姿矩阵
            dh_params: DH参数列表
        返回:
            关节位置列表（包括所有中间关节）
        """
        T = pelvis_pose.copy()
        joint_positions = [T[:3, 3]]  # 起始位置（骨盆）
        
        for dh in dh_params:
            T = T @ self.dh_transform(dh)
            joint_positions.append(T[:3, 3].copy())
        
        return joint_positions
    
    def calculate_hip_angle_from_imu(self, pelvis_imu: IMUData, 
                                     thigh_imu: IMUData) -> float:
        """
        从IMU数据计算髋关节角度
        
        参数:
            pelvis_imu: 骨盆IMU数据
            thigh_imu: 大腿IMU数据
        返回:
            髋关节角度（弧度）
        """
        R_pelvis = self.quaternion_to_rotation_matrix(pelvis_imu.quaternion)
        R_thigh = self.quaternion_to_rotation_matrix(thigh_imu.quaternion)
        
        # 计算相对旋转
        R_relative = R_pelvis.T @ R_thigh
        
        # 提取屈曲角度（绕Y轴）
        hip_angle = np.arctan2(R_relative[0, 2], R_relative[2, 2])
        
        return hip_angle
    
    def update(self, timestamp: float,
               pelvis_imu: IMUData,
               left_thigh_imu: IMUData,
               right_thigh_imu: IMUData,
               left_knee_encoder: EncoderData,
               right_knee_encoder: EncoderData,
               pelvis_position: np.ndarray = None):
        """
        更新运动学模型状态
        
        参数:
            timestamp: 时间戳
            pelvis_imu: 骨盆IMU数据
            left_thigh_imu: 左大腿IMU数据
            right_thigh_imu: 右大腿IMU数据
            left_knee_encoder: 左膝编码器数据
            right_knee_encoder: 右膝编码器数据
            pelvis_position: 骨盆位置（可选）
        """
        if pelvis_position is None:
            pelvis_position = np.array([0.0, 0.0, 0.0])
        
        # 构建骨盆位姿矩阵
        R_pelvis = self.quaternion_to_rotation_matrix(pelvis_imu.quaternion)
        pelvis_pose = np.eye(4)
        pelvis_pose[:3, :3] = R_pelvis
        pelvis_pose[:3, 3] = pelvis_position
        
        # 计算髋关节角度
        left_hip_angle = self.calculate_hip_angle_from_imu(pelvis_imu, left_thigh_imu)
        right_hip_angle = self.calculate_hip_angle_from_imu(pelvis_imu, right_thigh_imu)
        
        # 转换膝关节角度为弧度
        left_knee_angle = np.radians(left_knee_encoder.angle)
        right_knee_angle = np.radians(right_knee_encoder.angle)
        
        # 获取DH参数
        left_dh = self.get_leg_dh_params(left_hip_angle, left_knee_angle, 'left')
        right_dh = self.get_leg_dh_params(right_hip_angle, right_knee_angle, 'right')
        
        # 前向运动学
        left_positions = self.forward_kinematics_dh(pelvis_pose, left_dh)
        right_positions = self.forward_kinematics_dh(pelvis_pose, right_dh)
        
        # 存储历史数据
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
        """
        绘制3D骨架图
        
        参数:
            frame_idx: 要显示的帧索引（-1表示最后一帧）
        """
        if len(self.history['time']) == 0:
            print("没有数据可以绘制")
            return
        
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # 获取指定帧的数据
        left_pos = self.history['left_joint_positions'][frame_idx]
        right_pos = self.history['right_joint_positions'][frame_idx]
        
        # 提取关节位置
        # 左腿: 骨盆 -> 髋 -> 大腿中点 -> 膝 -> 小腿中点 -> 踝
        left_x = [p[0] for p in left_pos]
        left_y = [p[1] for p in left_pos]
        left_z = [p[2] for p in left_pos]
        
        # 右腿
        right_x = [p[0] for p in right_pos]
        right_y = [p[1] for p in right_pos]
        right_z = [p[2] for p in right_pos]
        
        # 绘制骨架
        ax.plot(left_x, left_y, left_z, 'b-o', linewidth=3, markersize=8, label='左腿')
        ax.plot(right_x, right_y, right_z, 'r-o', linewidth=3, markersize=8, label='右腿')
        
        # 绘制骨盆连线
        ax.plot([left_x[0], right_x[0]], 
                [left_y[0], right_y[0]], 
                [left_z[0], right_z[0]], 'g-', linewidth=4, label='骨盆')
        
        # 标注关节
        joints = ['骨盆', '髋', '大腿', '膝', '小腿', '踝']
        for i, joint in enumerate(joints):
            if i < len(left_pos):
                ax.text(left_x[i], left_y[i], left_z[i], f'L-{joint}', fontsize=8)
            if i < len(right_pos):
                ax.text(right_x[i], right_y[i], right_z[i], f'R-{joint}', fontsize=8)
        
        # 设置坐标轴
        ax.set_xlabel('X (米)', fontsize=12)
        ax.set_ylabel('Y (米)', fontsize=12)
        ax.set_zlabel('Z (米)', fontsize=12)
        ax.set_title(f'下肢骨架 (DH参数法) - 时间: {self.history["time"][frame_idx]:.2f}s', 
                     fontsize=14)
        ax.legend()
        
        # 设置相同的刻度范围
        all_x = left_x + right_x
        all_y = left_y + right_y
        all_z = left_z + right_z
        
        max_range = max(max(all_x)-min(all_x), 
                       max(all_y)-min(all_y), 
                       max(all_z)-min(all_z)) / 2.0
        
        mid_x = (max(all_x) + min(all_x)) * 0.5
        mid_y = (max(all_y) + min(all_y)) * 0.5
        mid_z = (max(all_z) + min(all_z)) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        ax.grid(True)
        plt.show()
    
    def animate_skeleton(self, interval=50, repeat=True, save_path=None):
        """
        播放3D骨架动画
        
        参数:
            interval: 帧间隔时间（毫秒）
            repeat: 是否循环播放
            save_path: 保存动画的路径（可选，如 'gait_animation.gif'）
        """
        from matplotlib.animation import FuncAnimation
        
        if len(self.history['time']) == 0:
            print("没有数据可以播放")
            return
        
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # 计算全局坐标范围（所有帧）
        all_positions = []
        for left_pos, right_pos in zip(self.history['left_joint_positions'], 
                                       self.history['right_joint_positions']):
            all_positions.extend(left_pos)
            all_positions.extend(right_pos)
        
        all_x = [p[0] for p in all_positions]
        all_y = [p[1] for p in all_positions]
        all_z = [p[2] for p in all_positions]
        
        max_range = max(max(all_x)-min(all_x), 
                       max(all_y)-min(all_y), 
                       max(all_z)-min(all_z)) / 2.0
        
        mid_x = (max(all_x) + min(all_x)) * 0.5
        mid_y = (max(all_y) + min(all_y)) * 0.5
        mid_z = (max(all_z) + min(all_z)) * 0.5
        
        # 初始化绘图元素
        left_line, = ax.plot([], [], [], 'b-o', linewidth=3, markersize=8, label='左腿')
        right_line, = ax.plot([], [], [], 'r-o', linewidth=3, markersize=8, label='右腿')
        pelvis_line, = ax.plot([], [], [], 'g-', linewidth=4, label='骨盆')
        
        # 关节标注文本
        joint_texts = []
        joints = ['骨盆', '髋', '大腿', '膝', '小腿', '踝']
        for _ in range(len(joints) * 2):  # 左右腿各有这些关节
            text = ax.text(0, 0, 0, '', fontsize=8)
            joint_texts.append(text)
        
        time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=12)
        
        # 设置固定的坐标轴范围和标签
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        ax.set_xlabel('X (米)', fontsize=12)
        ax.set_ylabel('Y (米)', fontsize=12)
        ax.set_zlabel('Z (米)', fontsize=12)
        ax.set_title('下肢步态动画 (DH参数法)', fontsize=14)
        ax.legend()
        ax.grid(True)
        
        def init():
            """初始化动画"""
            left_line.set_data([], [])
            left_line.set_3d_properties([])
            right_line.set_data([], [])
            right_line.set_3d_properties([])
            pelvis_line.set_data([], [])
            pelvis_line.set_3d_properties([])
            time_text.set_text('')
            for text in joint_texts:
                text.set_text('')
            return [left_line, right_line, pelvis_line, time_text] + joint_texts
        
        def update(frame):
            """更新每一帧"""
            # 获取当前帧的数据
            left_pos = self.history['left_joint_positions'][frame]
            right_pos = self.history['right_joint_positions'][frame]
            
            # 提取坐标
            left_x = [p[0] for p in left_pos]
            left_y = [p[1] for p in left_pos]
            left_z = [p[2] for p in left_pos]
            
            right_x = [p[0] for p in right_pos]
            right_y = [p[1] for p in right_pos]
            right_z = [p[2] for p in right_pos]
            
            # 更新线条
            left_line.set_data(left_x, left_y)
            left_line.set_3d_properties(left_z)
            
            right_line.set_data(right_x, right_y)
            right_line.set_3d_properties(right_z)
            
            pelvis_line.set_data([left_x[0], right_x[0]], [left_y[0], right_y[0]])
            pelvis_line.set_3d_properties([left_z[0], right_z[0]])
            
            # 更新时间文本
            time_text.set_text(f'时间: {self.history["time"][frame]:.2f}s | 帧: {frame}/{len(self.history["time"])-1}')
            
            # 更新关节标注
            text_idx = 0
            for i, joint in enumerate(joints):
                if i < len(left_pos):
                    joint_texts[text_idx].set_position((left_x[i], left_y[i]))
                    joint_texts[text_idx].set_3d_properties(left_z[i])
                    joint_texts[text_idx].set_text(f'L-{joint}')
                    text_idx += 1
                if i < len(right_pos):
                    joint_texts[text_idx].set_position((right_x[i], right_y[i]))
                    joint_texts[text_idx].set_3d_properties(right_z[i])
                    joint_texts[text_idx].set_text(f'R-{joint}')
                    text_idx += 1
            
            return [left_line, right_line, pelvis_line, time_text] + joint_texts
        
        # 创建动画
        num_frames = len(self.history['time'])
        anim = FuncAnimation(fig, update, frames=num_frames, 
                           init_func=init, blit=False,
                           interval=interval, repeat=repeat)
        
        # 保存动画（如果指定了路径）
        if save_path:
            print(f"正在保存动画到 {save_path}...")
            anim.save(save_path, writer='pillow', fps=1000//interval)
            print("动画保存完成！")
        
        plt.show()
        return anim
    
    def plot_joint_angles(self):
        """绘制关节角度随时间变化的图表"""
        if len(self.history['time']) == 0:
            print("没有数据可以绘制")
            return
        
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        time = self.history['time']
        
        # 髋关节角度
        axes[0].plot(time, self.history['left_hip_angle'], 'b-', label='左髋', linewidth=2)
        axes[0].plot(time, self.history['right_hip_angle'], 'r-', label='右髋', linewidth=2)
        axes[0].set_ylabel('髋关节角度 (度)', fontsize=12)
        axes[0].set_title('关节角度 (DH参数法)', fontsize=14)
        axes[0].legend()
        axes[0].grid(True)
        axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        # 膝关节角度
        axes[1].plot(time, self.history['left_knee_angle'], 'b-', label='左膝', linewidth=2)
        axes[1].plot(time, self.history['right_knee_angle'], 'r-', label='右膝', linewidth=2)
        axes[1].set_xlabel('时间 (秒)', fontsize=12)
        axes[1].set_ylabel('膝关节角度 (度)', fontsize=12)
        axes[1].legend()
        axes[1].grid(True)
        axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def print_dh_table(self, hip_angle: float, knee_angle: float, side: str):
        """
        打印DH参数表
        
        参数:
            hip_angle: 髋关节角度（度）
            knee_angle: 膝关节角度（度）
            side: 'left' 或 'right'
        """
        dh_params = self.get_leg_dh_params(np.radians(hip_angle), 
                                           np.radians(knee_angle), side)
        
        print(f"\n{side.upper()}腿DH参数表:")
        print("-" * 70)
        print(f"{'关节':<12} {'a (m)':<12} {'α (rad)':<12} {'d (m)':<12} {'θ (rad)':<12}")
        print("-" * 70)
        
        joint_names = ['骨盆-髋', '髋关节', '大腿', '膝关节', '小腿']
        for i, (name, dh) in enumerate(zip(joint_names, dh_params)):
            print(f"{name:<12} {dh.a:<12.4f} {dh.alpha:<12.4f} "
                  f"{dh.d:<12.4f} {dh.theta:<12.4f}")
        print("-" * 70)


# 使用示例
if __name__ == "__main__":
    # 创建运动学模型实例
    model = LowerLimbKinematicsDH(thigh_length=0.45, shank_length=0.43, hip_width=0.2)
    
    # 模拟数据：行走步态
    dt = 0.01  # 100Hz采样率
    duration = 2.0  # 2秒
    timestamps = np.arange(0, duration, dt)
    
    print("=" * 70)
    print("基于DH参数的下肢运动学模型")
    print("=" * 70)
    
    # 打印初始DH参数表
    model.print_dh_table(hip_angle=30, knee_angle=45, side='left')
    
    print("\n模拟步态数据...")
    for i, t in enumerate(timestamps):
        # 模拟IMU数据
        pelvis_imu = IMUData(
            accelerometer=np.array([0.1 * np.sin(2*np.pi*t), 9.81, 0]),
            gyroscope=np.array([0, 0.1 * np.cos(2*np.pi*t), 0]),
            quaternion=np.array([1, 0, 0.05*np.sin(2*np.pi*t), 0])
        )
        
        # 左腿（超前相位）
        phase_left = 2 * np.pi * t
        left_thigh_imu = IMUData(
            accelerometer=np.array([0.5 * np.sin(phase_left), 9.81, 0]),
            gyroscope=np.array([0, 0.3 * np.cos(phase_left), 0]),
            quaternion=np.array([np.cos(0.3*np.sin(phase_left)), 
                                0.3*np.sin(phase_left), 0, 
                                np.sin(0.3*np.sin(phase_left))])
        )
        
        # 右腿（滞后相位）
        phase_right = 2 * np.pi * t + np.pi/2
        right_thigh_imu = IMUData(
            accelerometer=np.array([0.5 * np.sin(phase_right), 0, 9.81]),
            gyroscope=np.array([0, 0.3 * np.cos(phase_right), 0]),
            quaternion=np.array([np.cos(0.3*np.sin(phase_right)), 
                                0.3*np.sin(phase_right), 0,
                                np.sin(0.3*np.sin(phase_right))])
        )
        
        # 模拟膝关节编码器数据
        left_knee_encoder = EncoderData(angle=0 + 35 * np.sin(phase_left))
        right_knee_encoder = EncoderData(angle=0 + 35 * np.sin(phase_right))
        
        # 更新模型
        result = model.update(
            t, pelvis_imu, left_thigh_imu, right_thigh_imu,
            left_knee_encoder, right_knee_encoder,
            pelvis_position=np.array([0, 0, 1.0])  # 骨盆高度1米
        )
        
        # 每0.5秒打印一次结果
        if int(t * 100) % 50 == 0:
            print(f"\n时间: {t:.2f}s")
            print(f"左髋角度: {result['left_hip_angle']:.1f}°")
            print(f"右髋角度: {result['right_hip_angle']:.1f}°")
            print(f"左膝角度: {result['left_knee_angle']:.1f}°")
            print(f"右膝角度: {result['right_knee_angle']:.1f}°")
    
    print("\n生成可视化图表...")
    
    # 绘制关节角度
    model.plot_joint_angles()
    
    # 播放3D骨架动画
    print("\n播放步态动画...")
    print("提示: 关闭窗口可继续显示静态图")
    
    # 动画播放（每帧50ms，循环播放）
    model.animate_skeleton(interval=50, repeat=True)
    
    # 可选：保存为GIF动画（取消注释下面这行）
    # model.animate_skeleton(interval=50, repeat=True, save_path='gait_animation.gif')
    
    # 绘制几个关键帧的静态图
    # print("\n显示关键帧...")
    # num_frames = len(model.history['time'])
    # frame_indices = [0, num_frames//2, -1]
    
    # for idx in frame_indices:
    #     model.plot_skeleton_3d(frame_idx=idx)