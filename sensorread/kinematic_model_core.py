# kinematics_core.py
import numpy as np
from scipy.spatial.transform import Rotation as R
from base.data_base import EncoderData, DHParameter, IMUData
from typing import List, Dict

# =========================
# 下肢 DH + FK 核心
# =========================
class LowerLimbKinematicsDH:
    def __init__(self,
                 thigh_length=0.45,
                 shank_length=0.43,
                 hip_width=0.20):

        self.thigh_length = thigh_length
        self.shank_length = shank_length
        self.hip_width = hip_width

        self.init_rot = {}
        self.initialized = False

        self._left_hist = []
        self._right_hist = []

        self.feature_history: List[Dict] = []

    # ---------- 基础 ----------
    def quat_to_R(self, q):
        return R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()

    def dh_T(self, dh: DHParameter):
        ct, st = np.cos(dh.theta), np.sin(dh.theta)
        ca, sa = np.cos(dh.alpha), np.sin(dh.alpha)
        return np.array([
            [ct, -st*ca,  st*sa, dh.a*ct],
            [st,  ct*ca, -ct*sa, dh.a*st],
            [0,      sa,     ca, dh.d],
            [0,       0,      0,   1]
        ])

    def fk(self, T0, dhs: List[DHParameter]):
        T = T0.copy()
        pos = [T[:3, 3].copy()]
        for dh in dhs:
            T = T @ self.dh_T(dh)
            pos.append(T[:3, 3].copy())
        return pos

    # ---------- 姿态 ----------
    def lock_init(self, name, q):
        if name not in self.init_rot:
            self.init_rot[name] = self.quat_to_R(q)

    def relative_R(self, name, q):
        self.lock_init(name, q)
        return self.init_rot[name].T @ self.quat_to_R(q)

    # ---------- 连续性 ----------
    def continuous(self, a, hist):
        if not hist:
            return a
        while a - hist[-1] > np.pi:
            a -= 2*np.pi
        while a - hist[-1] < -np.pi:
            a += 2*np.pi
        return a

    # ---------- 角度 ----------
    def hip_angle(self, R_pelvis, R_thigh):
        Rrel = R_pelvis.T @ R_thigh
        return np.arctan2(Rrel[0, 2], Rrel[2, 2])

    # ---------- DH ----------
    def leg_dh(self, hip, knee, side):
        off = self.hip_width/2 if side == "left" else -self.hip_width/2
        return [
            DHParameter(0, 0, off, 0),
            DHParameter(0, 0, 0, hip),
            DHParameter(self.thigh_length, 0, 0, 0),
            DHParameter(0, 0, 0, knee),
            DHParameter(self.shank_length, 0, 0, 0)
        ]

    # =========================
    # 主更新接口
    # =========================
    def update(self,
               pelvis_imu: IMUData,
               left_thigh_imu: IMUData,
               right_thigh_imu: IMUData,
               left_enc: EncoderData,
               right_enc: EncoderData):

        self.lock_init("pelvis", pelvis_imu.quaternion)
        self.lock_init("left", left_thigh_imu.quaternion)
        self.lock_init("right", right_thigh_imu.quaternion)

        Rp = self.relative_R("pelvis", pelvis_imu.quaternion)
        Rl = self.relative_R("left", left_thigh_imu.quaternion)
        Rr = self.relative_R("right", right_thigh_imu.quaternion)

        lh = self.continuous(self.hip_angle(Rp, Rl), self._left_hist)
        rh = self.continuous(self.hip_angle(Rp, Rr), self._right_hist)

        self._left_hist.append(lh)
        self._right_hist.append(rh)

        T0 = np.eye(4)
        T0[:3, :3] = Rp
        T0[:3, 3] = np.zeros(3)

        left_dh = self.leg_dh(lh, left_enc.angle, "left")
        right_dh = self.leg_dh(rh, right_enc.angle, "right")

        left_fk = self.fk(T0, left_dh)
        right_fk = self.fk(T0, right_dh)

        feat = {
            "left_hip": lh,
            "right_hip": rh,
            "left_knee": left_enc.angle,
            "right_knee": right_enc.angle,
            "dh": {},
            "fk": {}
        }

        for i, dh in enumerate(left_dh):
            feat["dh"][f"L{i}_theta"] = dh.theta
        for i, dh in enumerate(right_dh):
            feat["dh"][f"R{i}_theta"] = dh.theta

        for j, p in enumerate(left_fk):
            feat["fk"][f"L_fk_{j}_x"] = p[0]
            feat["fk"][f"L_fk_{j}_y"] = p[1]
            feat["fk"][f"L_fk_{j}_z"] = p[2]

        for j, p in enumerate(right_fk):
            feat["fk"][f"R_fk_{j}_x"] = p[0]
            feat["fk"][f"R_fk_{j}_y"] = p[1]
            feat["fk"][f"R_fk_{j}_z"] = p[2]

        # self.feature_history.append(feat)

        return feat