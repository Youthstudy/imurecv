# data/feature_detect.py
import re

def detect_feature_groups(columns):
    imu = []
    dh = []
    fk = []
    joint = []
    other = []

    for c in columns:
        name = c.lower()

        # ---- 1. IMU ----
        if ("acc" in name or "gyro" in name or "quat" in name) and \
           ("pelvis" in name or "left_thigh" in name or "right_thigh" in name):
            imu.append(c)
            continue

        # ---- 2. DH 参数 ----
        # e.g., L0_a, R3_theta_cos
        if re.match(r"[lr][0-4]_.*", name):
            dh.append(c)
            continue

        # ---- 3. FK 关节点坐标 ----
        # e.g., L_hip_x, R_knee_y, L_ankle_z
        if re.match(r"[lr]_(hip|thigh|knee|shank|ankle)_[xyz]", name):
            fk.append(c)
            continue

        # ---- 4. Joint angles ----
        if "knee_angle" in name:
            joint.append(c)
            continue

        # ---- 5. Others ----
        other.append(c)

    return {
        "imu": imu,
        "dh": dh,
        "fk": fk,
        "joint": joint,
        "other": other,
    }
