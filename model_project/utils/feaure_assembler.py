import numpy as np

class FeatureAssembler:
    def __init__(self, config: dict):
        self.use_imu   = config.get("USE_IMU", False)
        self.use_dh    = config.get("USE_DH", False)
        self.use_fk    = config.get("USE_FK", False)
        self.use_joint = config.get("USE_JOINT", False)
        self.use_other = config.get("USE_OTHER", False)

    def assemble_at(self, cache: dict, t: int) -> np.ndarray:
        """
        从 cache 中组装 t 时刻的特征向量
        """
        feats = []

        # ---------- IMU ----------
        if self.use_imu:
            feats += self._imu_features(cache["imu"], t)

        # ---------- Joint ----------
        if self.use_joint:
            feats += self._joint_features(cache["joint"], t)

        # ---------- DH ----------
        if self.use_dh:
            feats += self._dh_features(cache["dh"], t)

        # ---------- FK ----------
        if self.use_fk:
            feats += self._fk_features(cache["fk"], t)

        # ---------- Other ----------
        if self.use_other:
            feats += self._other_features(cache.get("other", {}), t)

        return np.asarray(feats, dtype=np.float32)
    
    def _imu_features(self, imu_cache: dict, t: int):
        f = []
        for seg in ["pelvis", "left", "right"]:
            imu = imu_cache[seg]
            f += imu.acc[t].tolist()
            f += imu.gyro[t].tolist()
            f += imu.quat[t].tolist()
        return f
    
    def _joint_features(self, joint_cache: dict, t: int):
        f = []
        for k in ["left_knee", "right_knee"]:
            enc = joint_cache[k]
            f.append(enc.angle[t])
            f.append(enc.velocity[t])
        return f
    
    def _dh_features(self, dh_cache: dict, t: int):
        f = []
        for side in ["left", "right"]:
            f += dh_cache[side][t].tolist()
        return f
    
    def _fk_features(self, fk_cache: dict, t: int):
        f = []
        for side in ["left", "right"]:
            f += fk_cache[side][t].tolist()
        return f
    
    def assemble_now(self, cache: dict) -> np.ndarray:
        feats = []

        if self.use_imu:
            feats += self._imu(cache["imu"])

        if self.use_joint:
            feats += self._joint(cache["joint"])

        if self.use_dh:
            feats += self._dh(cache["dh"])

        if self.use_fk:
            feats += self._fk(cache["fk"])

        return np.asarray(feats, dtype=np.float32)
    
    def _imu(self, imu_cache: dict):
        f = []
        for seg in ["pelvis", "left", "right"]:
            imu = imu_cache[seg]
            f += imu.acc.tolist()
            f += imu.gyro.tolist()
            f += imu.quat.tolist()
        return f
    
    def _joint(self, joint_cache: dict):
        f = []
        for k in ["left_knee", "right_knee"]:
            enc = joint_cache[k]
            f += enc.angle.tolist()
            # f += enc.velocity.tolist()
        return f
    
    def _dh(self, dh_cache):
        f = []

        for side in ["left", "right"]:
            dh_list = dh_cache[side]
            for dh in dh_list:
                f.append(dh.theta)

        return f

    
    def _fk(self, fk_cache: dict):
        f = []
        for side in ["left", "right"]:
            f += fk_cache[side].tolist()
        return f



