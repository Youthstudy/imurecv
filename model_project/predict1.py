# predict.py - 简化版步态推理器
# 复用已有模型和运动学模块，根据 JSON 配置自动整合特征

import json
import threading
import time
import pickle
import torch
import numpy as np
from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional, Callable, List
from pathlib import Path

# ============ 导入已有模块 ============
from .models.model_lstm import LSTMRegressor
from .models.model_cnn import CNNRegressor
from .models.model_svm import SVMRegressor

from base.data_base import IMUData, EncoderData, DHParameter
from .utils.kinematic_model_core import LowerLimbKinematicsDH


# ============ 数据结构 ============

# ==================== 模型加载 ====================
class ModelLoader:
    @staticmethod
    def infer_params(state_dict: dict, model_type: str) -> dict:
        if model_type == "lstm":
            w_ih = state_dict.get("lstm.weight_ih_l0")
            w_fc = state_dict.get("fc.weight")
            layers = sum(1 for k in state_dict if k.startswith("lstm.weight_ih_l"))
            return {
                "input_dim": w_ih.shape[1] if w_ih is not None else 0,
                "hidden": w_ih.shape[0] // 4 if w_ih is not None else 128,
                "layers": layers,
                "output_dim": w_fc.shape[0] if w_fc is not None else 3
            }
        elif model_type == "cnn":
            conv1 = state_dict.get("net.0.weight")
            fc = state_dict.get("fc.weight")
            return {
                "input_dim": conv1.shape[1] if conv1 is not None else 0,
                "output_dim": fc.shape[0] if fc is not None else 3
            }
        return {}
    
    @classmethod
    def load(cls, path: str, model_type: str, device: torch.device):
        model_type = model_type.lower()
        
        if model_type == "svm":
            with open(path, 'rb') as f:
                data = pickle.load(f)
            model = SVMRegressor()
            model.models = data if isinstance(data, list) else data.get('models', data)
            return model
        
        ckpt = torch.load(path, map_location=device, weights_only=False)
        if hasattr(ckpt, 'eval'):
            ckpt.to(device).eval()
            return ckpt
        
        state = ckpt.get('state_dict', ckpt)
        params = cls.infer_params(state, model_type)
        model = LSTMRegressor(**params) if model_type == "lstm" else CNNRegressor(**params)
        model.load_state_dict(state)
        model.to(device).eval()
        return model


# ==================== 核心推理器 ====================
class GaitPredictor:
    IMU_DEVICES = ["pelvis", "left_thigh", "right_thigh"]
    JOINTS = ["left_knee", "right_knee"]
    
    def __init__(self, config_path: str, device: str = "auto"):
        with open(config_path, 'r') as f:
            self.cfg = json.load(f)
        
        self.device = torch.device(
            "cuda" if device == "auto" and torch.cuda.is_available() else 
            device if device != "auto" else "cpu"
        )
        
        self.model_type = self.cfg.get("MODEL_TYPE", "lstm").lower()
        self.window_size = self.cfg.get("WINDOW", 50)
        self.output_cols = self.cfg.get("OUTPUT_COLS", ["gait_phase", "left_knee_angle", "right_knee_angle"])
        
        self.use_imu = self.cfg.get("USE_IMU", False)
        self.use_joint = self.cfg.get("USE_JOINT", False)
        self.use_dh = self.cfg.get("USE_DH", False)
        self.use_fk = self.cfg.get("USE_FK", False)
        
        self.cache: Dict[str, dict] = {"imu": {}, "joint": {}}
        self.window = deque(maxlen=self.window_size)
        self.kin = LowerLimbKinematicsDH() if (self.use_dh or self.use_fk) else None
        self.model = self._load_model()
    
    def _load_model(self):
        path = self.cfg.get("MODEL_PATH", "")
        if not path or not Path(path).exists():
            return None
        return ModelLoader.load(path, self.model_type, self.device)
    
    def update(self, imu: Dict[str, IMUData] = None, joint: Dict[str, EncoderData] = None):
        if imu:
            self.cache["imu"].update(imu)
        if joint:
            self.cache["joint"].update(joint)
    
    def update_imu(self, device_id: str, data: IMUData):
        self.cache["imu"][device_id] = data
    
    def update_joint(self, joint_id: str, data: EncoderData):
        self.cache["joint"][joint_id] = data
    
    def _compute_kin(self) -> Optional[dict]:
        if not self.kin:
            return None
        for d in self.IMU_DEVICES:
            if d not in self.cache["imu"]:
                return None
        for j in self.JOINTS:
            if j not in self.cache["joint"]:
                return None
        try:
            result = self.kin.update(
                self.cache["imu"]["pelvis"],
                self.cache["imu"]["left_thigh"],
                self.cache["imu"]["right_thigh"],
                self.cache["joint"]["left_knee"],
                self.cache["joint"]["right_knee"]
            )
            # 确保 dh 和 fk 键存在
            if result and "dh" not in result:
                result["dh"] = {}
            if result and "fk" not in result:
                result["fk"] = {}
            return result
        except Exception as e:
            print(f"运动学计算错误: {e}")
            return None
    
    def _extract_features(self) -> Optional[np.ndarray]:
        features = []
        kin_feat = self._compute_kin() if self.kin else None
        
        if self.use_imu:
            for dev in self.IMU_DEVICES:
                if dev not in self.cache["imu"]:
                    return None
                features.append(self.cache["imu"][dev].to_array())
        
        if self.use_joint:
            for jnt in self.JOINTS:
                if jnt not in self.cache["joint"]:
                    return None
                features.append(self.cache["joint"][jnt].to_array())
        
        if self.use_dh:
            if kin_feat and "dh" in kin_feat and kin_feat["dh"]:
                dh_vals = [v for _, v in sorted(kin_feat["dh"].items())]
                if dh_vals:
                    features.append(np.array(dh_vals))
            else:
                # DH 被启用但无法计算，可能是 kinematic_model_core.py 有 bug
                # 尝试直接从 kin 对象获取最近的 DH 值
                if self.kin and hasattr(self.kin, '_left_hist') and self.kin._left_hist:
                    # 使用缓存数据手动构建 DH
                    try:
                        lh = self.kin._left_hist[-1] if self.kin._left_hist else 0.0
                        rh = self.kin._right_hist[-1] if self.kin._right_hist else 0.0
                        lk = self.cache["joint"]["left_knee"].angle if "left_knee" in self.cache["joint"] else 0.0
                        rk = self.cache["joint"]["right_knee"].angle if "right_knee" in self.cache["joint"] else 0.0
                        
                        # 构建 DH theta 值 (左腿5个 + 右腿5个 = 10个)
                        # leg_dh 返回: [offset, hip, 0, knee, 0]
                        left_dh_theta = [0.0, lh, 0.0, lk, 0.0]
                        right_dh_theta = [0.0, rh, 0.0, rk, 0.0]
                        dh_vals = left_dh_theta + right_dh_theta
                        features.append(np.array(dh_vals))
                    except Exception as e:
                        print(f"手动构建DH失败: {e}")
                        return None
                else:
                    print("警告: USE_DH=true 但 DH 特征为空，请检查 kinematic_model_core.py")
                    return None
        
        if self.use_fk:
            if kin_feat and "fk" in kin_feat and kin_feat["fk"]:
                fk_vals = [v for _, v in sorted(kin_feat["fk"].items())]
                if fk_vals:
                    features.append(np.array(fk_vals))
            else:
                print("警告: USE_FK=true 但 FK 特征为空")
                return None
        
        return np.concatenate(features).astype(np.float32) if features else None
    
    def predict(self) -> Optional[Dict[str, float]]:
        feat = self._extract_features()
        if feat is None:
            return None
        
        self.window.append(feat)
        if len(self.window) < self.window_size:
            return None
        
        if not self.model:
            return {"feature_dim": len(feat), "window_ready": True}
        
        data = np.array(self.window)
        
        if self.model_type == "svm":
            out = self.model.predict(data.reshape(1, -1))[0]
        else:
            x = torch.FloatTensor(data).unsqueeze(0).to(self.device)
            with torch.no_grad():
                out = self.model(x).cpu().numpy()[0]
        
        return {col: float(out[i]) for i, col in enumerate(self.output_cols) if i < len(out)}
    
    def reset(self):
        self.cache = {"imu": {}, "joint": {}}
        self.window.clear()
    
    def info(self) -> dict:
        return {
            "model_type": self.model_type,
            "device": str(self.device),
            "window": f"{len(self.window)}/{self.window_size}",
            "output_cols": self.output_cols
        }


# ==================== 推理线程 ====================
class InferenceThread:
    def __init__(self, config_path: str, freq: float = 100.0, device: str = "auto"):
        self.predictor = GaitPredictor(config_path, device)
        self.freq = freq
        self.interval = 1.0 / freq
        
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()
        
        self._callbacks: List[Callable] = []
        self._result: Optional[dict] = None
        self._result_lock = threading.Lock()
        
        self._stats = {"frame_count": 0, "last_fps": 0.0}
        self._fps_cnt = 0
        self._fps_t = time.perf_counter()
    
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
    
    def _loop(self):
        while self._running:
            t0 = time.perf_counter()
            
            with self._lock:
                result = self.predictor.predict()
            
            if result:
                with self._result_lock:
                    self._result = result
                for cb in self._callbacks:
                    try:
                        cb(result)
                    except Exception as e:
                        pass
            
            self._fps_cnt += 1
            self._stats["frame_count"] += 1
            now = time.perf_counter()
            if now - self._fps_t >= 1.0:
                self._stats["last_fps"] = self._fps_cnt / (now - self._fps_t)
                self._fps_cnt = 0
                self._fps_t = now
            
            elapsed = time.perf_counter() - t0
            if elapsed < self.interval:
                time.sleep(self.interval - elapsed)
    
    def update(self, imu: Dict[str, IMUData] = None, joint: Dict[str, EncoderData] = None):
        with self._lock:
            self.predictor.update(imu, joint)
    
    def update_imu(self, device_id: str, data: IMUData):
        with self._lock:
            self.predictor.update_imu(device_id, data)
    
    def update_joint(self, joint_id: str, data: EncoderData):
        with self._lock:
            self.predictor.update_joint(joint_id, data)
    
    def set_callback(self, callback: Callable):
        if callback not in self._callbacks:
            self._callbacks.append(callback)
    
    def get_result(self) -> Optional[dict]:
        with self._result_lock:
            return self._result.copy() if self._result else None
    
    def get_stats(self) -> dict:
        return self._stats.copy()
    
    def info(self) -> dict:
        return {"inferencer": self.predictor.info(), "freq": self.freq}
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, *_):
        self.stop()

"""
# ============ 模型加载 ============
class ModelLoader:
    # 统一模型加载
    
    @staticmethod
    def infer_params(state_dict: dict, model_type: str) -> dict:
        if model_type == "lstm":
            w_ih = state_dict.get("lstm.weight_ih_l0")
            w_fc = state_dict.get("fc.weight")
            layers = sum(1 for k in state_dict if k.startswith("lstm.weight_ih_l"))
            return {
                "input_dim": w_ih.shape[1] if w_ih is not None else 0,
                "hidden": w_ih.shape[0] // 4 if w_ih is not None else 128,
                "layers": layers,
                "output_dim": w_fc.shape[0] if w_fc is not None else 3
            }
        elif model_type == "cnn":
            conv1 = state_dict.get("net.0.weight")
            fc = state_dict.get("fc.weight")
            return {
                "input_dim": conv1.shape[1] if conv1 is not None else 0,
                "output_dim": fc.shape[0] if fc is not None else 3
            }
        return {}
    
    @classmethod
    def load(cls, path: str, model_type: str, device: torch.device):
        model_type = model_type.lower()
        
        if model_type == "svm":
            with open(path, 'rb') as f:
                data = pickle.load(f)
            model = SVMRegressor()
            model.models = data if isinstance(data, list) else data.get('models', data)
            print(f"✓ SVM: {path}")
            return model
        
        ckpt = torch.load(path, map_location=device, weights_only=False)
        if hasattr(ckpt, 'eval'):
            ckpt.to(device).eval()
            print(f"✓ {model_type.upper()}: {path}")
            return ckpt
        
        state = ckpt.get('state_dict', ckpt)
        params = cls.infer_params(state, model_type)
        model = LSTMRegressor(**params) if model_type == "lstm" else CNNRegressor(**params)
        model.load_state_dict(state)
        model.to(device).eval()
        print(f"✓ {model_type.upper()}: {path} | {params}")
        return model


# ============ 核心推理器 ============
class GaitPredictor:
    # 步态推理核心 - 根据 JSON 配置自动选择特征
    
    IMU_DEVICES = ["pelvis", "left_thigh", "right_thigh"]
    JOINTS = ["left_knee", "right_knee"]
    
    def __init__(self, config_path: str, device: str = "auto"):
        with open(config_path, 'r') as f:
            self.cfg = json.load(f)
        
        self.device = torch.device(
            "cuda" if device == "auto" and torch.cuda.is_available() else 
            device if device != "auto" else "cpu"
        )
        
        self.model_type = self.cfg.get("MODEL_TYPE", "lstm").lower()
        self.window_size = self.cfg.get("WINDOW", 50)
        self.output_cols = self.cfg.get("OUTPUT_COLS", ["gait_phase", "left_knee_angle", "right_knee_angle"])
        
        # 特征开关
        self.use_imu = self.cfg.get("USE_IMU", False)
        self.use_joint = self.cfg.get("USE_JOINT", False)
        self.use_dh = self.cfg.get("USE_DH", False)
        self.use_fk = self.cfg.get("USE_FK", False)
        
        print(f"特征: IMU={self.use_imu}, JOINT={self.use_joint}, DH={self.use_dh}, FK={self.use_fk}")
        
        # 数据缓存
        self.cache: Dict[str, dict] = {"imu": {}, "joint": {}}
        self.window = deque(maxlen=self.window_size)
        
        # 运动学（DH/FK 需要）
        self.kin = LowerLimbKinematicsDH() if (self.use_dh or self.use_fk) else None
        
        # 模型
        self.model = self._load_model()
    
    def _load_model(self):
        path = self.cfg.get("MODEL_PATH", "")
        if not path or not Path(path).exists():
            print("⚠ 无模型，特征提取模式")
            return None
        return ModelLoader.load(path, self.model_type, self.device)
    
    def update(self, imu: Dict[str, IMUData] = None, joint: Dict[str, EncoderData] = None):
        # 更新传感器数据
        if imu:
            self.cache["imu"].update(imu)
        if joint:
            self.cache["joint"].update(joint)
    
    def _compute_kin(self) -> Optional[dict]:
        # 计算运动学特征
        if not self.kin:
            return None
        
        for d in self.IMU_DEVICES:
            if d not in self.cache["imu"]:
                return None
        for j in self.JOINTS:
            if j not in self.cache["joint"]:
                return None
        
        try:
            return self.kin.update(
                self.cache["imu"]["pelvis"],
                self.cache["imu"]["left_thigh"],
                self.cache["imu"]["right_thigh"],
                self.cache["joint"]["left_knee"],
                self.cache["joint"]["right_knee"]
            )
        except:
            return None
    
    def _extract_features(self) -> Optional[np.ndarray]:
        # 根据配置提取特征
        features = []
        kin_feat = self._compute_kin() if self.kin else None
        
        if self.use_imu:
            for dev in self.IMU_DEVICES:
                if dev not in self.cache["imu"]:
                    return None
                features.append(self.cache["imu"][dev].to_array())
        
        if self.use_joint:
            for jnt in self.JOINTS:
                if jnt not in self.cache["joint"]:
                    return None
                features.append(self.cache["joint"][jnt].to_array())
        
        if self.use_dh and kin_feat and "dh" in kin_feat:
            dh_vals = [v for _, v in sorted(kin_feat["dh"].items())]
            if dh_vals:
                features.append(np.array(dh_vals))
        
        if self.use_fk and kin_feat and "fk" in kin_feat:
            fk_vals = [v for _, v in sorted(kin_feat["fk"].items())]
            if fk_vals:
                features.append(np.array(fk_vals))
        
        return np.concatenate(features).astype(np.float32) if features else None
    
    def predict(self) -> Optional[Dict[str, float]]:
        # 执行推理
        feat = self._extract_features()
        if feat is None:
            return None
        
        self.window.append(feat)
        if len(self.window) < self.window_size:
            return None
        
        if not self.model:
            return {"feature_dim": len(feat), "window_ready": True}
        
        data = np.array(self.window)
        
        if self.model_type == "svm":
            out = self.model.predict(data.reshape(1, -1))[0]
        else:
            x = torch.FloatTensor(data).unsqueeze(0).to(self.device)
            with torch.no_grad():
                out = self.model(x).cpu().numpy()[0]
        
        return {col: float(out[i]) for i, col in enumerate(self.output_cols) if i < len(out)}
    
    def reset(self):
        self.cache = {"imu": {}, "joint": {}}
        self.window.clear()


# ============ 实时推理线程 ============
class InferenceThread:
    # 线程化实时推理
    
    def __init__(self, config_path: str, freq: float = 100.0, device: str = "auto"):
        self.predictor = GaitPredictor(config_path, device)
        self.freq = freq
        self.interval = 1.0 / freq
        
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()
        
        self._callbacks: List[Callable] = []
        self._result: Optional[dict] = None
        self._result_lock = threading.Lock()
        
        self._stats = {"frames": 0, "fps": 0.0}
        self._fps_cnt = 0
        self._fps_t = time.perf_counter()
    
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"✓ 推理线程 @ {self.freq} Hz")
    
    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        print("✓ 线程停止")
    
    def _loop(self):
        while self._running:
            t0 = time.perf_counter()
            
            with self._lock:
                result = self.predictor.predict()
            
            if result:
                with self._result_lock:
                    self._result = result
                for cb in self._callbacks:
                    try:
                        cb(result)
                    except Exception as e:
                        print(f"回调错误: {e}")
            
            self._fps_cnt += 1
            self._stats["frames"] += 1
            now = time.perf_counter()
            if now - self._fps_t >= 1.0:
                self._stats["fps"] = self._fps_cnt / (now - self._fps_t)
                self._fps_cnt = 0
                self._fps_t = now
            
            elapsed = time.perf_counter() - t0
            if elapsed < self.interval:
                time.sleep(self.interval - elapsed)
    
    def update(self, imu: Dict[str, IMUData] = None, joint: Dict[str, EncoderData] = None):
        with self._lock:
            self.predictor.update(imu, joint)
    
    def on_result(self, callback: Callable):
        self._callbacks.append(callback)
    
    def get_result(self) -> Optional[dict]:
        with self._result_lock:
            return self._result.copy() if self._result else None
    
    def get_stats(self) -> dict:
        return self._stats.copy()
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, *_):
        self.stop()
"""
# ============================================================
# 使用例程
# ============================================================
"""
配置文件格式 (xxx_config.json):
{
    "MODEL_PATH": "model_project/result/lstm_xxx.pth",
    "MODEL_TYPE": "lstm",
    "WINDOW": 50,
    "USE_IMU": true,
    "USE_JOINT": true,
    "USE_DH": true,
    "USE_FK": false,
    "OUTPUT_COLS": ["gait_phase", "left_knee_angle", "right_knee_angle"]
}

例程 1 - 线程模式（推荐）:

    from predict import InferenceThread, IMUData, EncoderData
    import numpy as np
    
    def on_result(r):
        print(f"相位: {r['gait_phase']:.3f}, 左膝: {r['left_knee_angle']:.2f}")
    
    with InferenceThread("lstm_config.json", freq=100) as thread:
        thread.on_result(on_result)
        
        while running:
            thread.update(
                imu={
                    "pelvis": IMUData(acc, gyro, quat),
                    "left_thigh": IMUData(acc, gyro, quat),
                    "right_thigh": IMUData(acc, gyro, quat)
                },
                joint={
                    "left_knee": EncoderData(angle, velocity, t_ff),
                    "right_knee": EncoderData(angle, velocity, t_ff)
                }
            )
            time.sleep(0.01)
        
        print(thread.get_stats())


例程 2 - 非线程模式:

    from predict import GaitPredictor, IMUData, EncoderData
    
    predictor = GaitPredictor("lstm_config.json")
    
    for frame in sensor_stream:
        predictor.update(imu={...}, joint={...})
        result = predictor.predict()
        if result:
            print(result)


例程 3 - 轮询结果:

    thread = InferenceThread("config.json", freq=100)
    thread.start()
    
    while running:
        thread.update(imu=..., joint=...)
        result = thread.get_result()  # 非阻塞获取最新结果
        if result:
            do_something(result)
        time.sleep(0.01)
    
    thread.stop()
"""