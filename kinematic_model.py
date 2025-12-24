import numpy as np
from dataclasses import dataclass
from typing import List, Dict
from scipy.spatial.transform import Rotation as R
import pandas as pd
@dataclass
class IMUData:
    accelerometer: np.ndarray
    gyroscope: np.ndarray
    quaternion: np.ndarray  # [w, x, y, z]
@dataclass
class EncoderData:
    angle: float  # rad
    velocity: float = 0.0
    t_ff: float = 0.0

@dataclass
class DHParameter:
    a: float
    alpha: float
    d: float
    theta: float


class LowerLimbKinematicsRealtime:
    """
    无绘图 / 无动画
    专用于：IMU + 编码器 → DH + FK → 特征
    """

    def __init__(self, thigh_length=0.45, shank_length=0.43, hip_width=0.2):
        self.thigh_length = thigh_length
        self.shank_length = shank_length
        self.hip_width = hip_width

        self.init_rot = {}
        self.initialized = False

        self._lh_hist: List[float] = []
        self._rh_hist: List[float] = []

    # ------------------ 基础数学 ------------------
    def _quat2R(self, q):
        return R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()

    def _dh_T(self, dh: DHParameter):
        ct, st = np.cos(dh.theta), np.sin(dh.theta)
        ca, sa = np.cos(dh.alpha), np.sin(dh.alpha)
        return np.array([
            [ct, -st*ca,  st*sa, dh.a*ct],
            [st,  ct*ca, -ct*sa, dh.a*st],
            [0,     sa,     ca,    dh.d],
            [0,     0,      0,     1]
        ])

    def _fk(self, T0, dhs):
        T = T0.copy()
        pos = [T[:3, 3].copy()]
        for dh in dhs:
            T = T @ self._dh_T(dh)
            pos.append(T[:3, 3].copy())
        return pos

    # ------------------ 姿态处理 ------------------
    def _lock_init(self, name, q):
        if name not in self.init_rot:
            self.init_rot[name] = self._quat2R(q)

    def _R_rel(self, name, q):
        self._lock_init(name, q)
        return self.init_rot[name].T @ self._quat2R(q)

    def _unwrap(self, a, hist):
        if not hist:
            return a
        while a - hist[-1] > np.pi:
            a -= 2*np.pi
        while a - hist[-1] < -np.pi:
            a += 2*np.pi
        return a

    def _hip_angle(self, Rp, Rt):
        Rrel = Rp.T @ Rt
        return np.arctan2(Rrel[0, 2], Rrel[2, 2])

    # ------------------ DH 构造 ------------------
    def _leg_dh(self, hip, knee, side):
        off = self.hip_width/2 if side == "left" else -self.hip_width/2
        return [
            DHParameter(0, 0, off, 0),
            DHParameter(0, 0, 0, hip),
            DHParameter(self.thigh_length, 0, 0, 0),
            DHParameter(0, 0, 0, knee),
            DHParameter(self.shank_length, 0, 0, 0)
        ]
    
    def step(self,
             pelvis_imu: IMUData,
             left_thigh_imu: IMUData,
             right_thigh_imu: IMUData,
             left_knee: EncoderData,
             right_knee: EncoderData,
             timestamp: float = 0.0) -> Dict:
        """
        输入一帧传感器数据
        输出：DH + FK 特征（dict）
        """

        # 相对旋转
        Rp = self._R_rel("pelvis", pelvis_imu.quaternion)
        Rl = self._R_rel("left_thigh", left_thigh_imu.quaternion)
        Rr = self._R_rel("right_thigh", right_thigh_imu.quaternion)

        # 髋角
        lh = self._unwrap(self._hip_angle(Rp, Rl), self._lh_hist)
        rh = self._unwrap(self._hip_angle(Rp, Rr), self._rh_hist)
        self._lh_hist.append(lh)
        self._rh_hist.append(rh)

        # DH
        left_dh = self._leg_dh(lh, left_knee.angle, "left")
        right_dh = self._leg_dh(rh, right_knee.angle, "right")

        # 骨盆位姿
        T0 = np.eye(4)
        T0[:3, :3] = Rp
        T0[:3, 3] = np.zeros(3)

        Lpos = self._fk(T0, left_dh)
        Rpos = self._fk(T0, right_dh)
        root = Lpos[0]

        feat = {}

        # ===== DH 特征 =====
        for i, dh in enumerate(left_dh):
            feat[f"L{i}_theta"] = dh.theta
            feat[f"L{i}_sin"] = np.sin(dh.theta)
            feat[f"L{i}_cos"] = np.cos(dh.theta)

        for i, dh in enumerate(right_dh):
            feat[f"R{i}_theta"] = dh.theta
            feat[f"R{i}_sin"] = np.sin(dh.theta)
            feat[f"R{i}_cos"] = np.cos(dh.theta)

        # ===== FK 特征（root-relative）=====
        names = ["hip", "thigh", "knee", "shank", "ankle"]
        for i, n in enumerate(names):
            p = Lpos[i] - root
            feat[f"L_{n}_x"], feat[f"L_{n}_y"], feat[f"L_{n}_z"] = p

            p = Rpos[i] - root
            feat[f"R_{n}_x"], feat[f"R_{n}_y"], feat[f"R_{n}_z"] = p

        # ===== 关节角 =====
        feat["left_hip_angle"] = lh
        feat["right_hip_angle"] = rh
        feat["left_knee_angle"] = left_knee.angle
        feat["right_knee_angle"] = right_knee.angle
        # ===== 记录到 buffer（不影响实时）=====
        if self.enable_record:
            feat["time"] = timestamp
            self.feature_buffer.append(feat)

        return feat
    
    def save_csv(self, path: str):
        """
        将已缓存的特征保存为 CSV
        """
        if not self.feature_buffer:
            print("⚠️ feature_buffer 为空，未保存 CSV")
            return

        df = pd.DataFrame(self.feature_buffer)
        df.to_csv(path, index=False, encoding="utf-8")
        print(f"✅ 已保存 CSV: {path}")

    def reset(self):
        """清空历史（用于新实验段）"""
        self.init_rot.clear()
        self._lh_hist.clear()
        self._rh_hist.clear()
        self.feature_buffer.clear()




