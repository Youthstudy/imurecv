"""
传感器读取控制界面 - 集成步态推理（简化版）
"""

import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import os
import logging
from datetime import datetime
from typing import Optional, Dict, Any, Callable, List
import copy
import json
import pickle
import torch
import numpy as np
from collections import deque
from dataclasses import dataclass
from pathlib import Path


# ==================== 导入已有模块 ====================
from model_project.models.model_lstm import LSTMRegressor
from model_project.models.model_cnn import CNNRegressor
from model_project.models.model_svm import SVMRegressor
from model_project.utils.kinematic_model_core import LowerLimbKinematicsDH, EncoderData as KinEncoderData
import sensorread.imu_adapter as imu_adp
import sensorread.encoder_adapter as enc_adp
import sensorread.bt_connect as btc
import sensorread.MCU2PC as m2p
from base.data_base import IMUData, EncoderData, DHParameter
from model_project.predict1 import GaitPredictor, InferenceThread
# ==================== 数据结构 ====================





# ==================== 日志配置 ====================
LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
LOG_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"

def setup_logging(log_file: Optional[str] = None, level: int = logging.DEBUG) -> logging.Logger:
    logger = logging.getLogger("SensorUI")
    logger.setLevel(level)
    
    if not logger.handlers:
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(logging.Formatter(LOG_FORMAT, LOG_DATE_FORMAT))
        logger.addHandler(console_handler)
        
        if log_file:
            file_handler = logging.FileHandler(log_file, encoding='utf-8')
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(logging.Formatter(LOG_FORMAT, LOG_DATE_FORMAT))
            logger.addHandler(file_handler)
    
    return logger


# ==================== 样式配置 ====================
class ThemeConfig:
    BG_PRIMARY = "#f5f5f5"
    BG_SECONDARY = "#ffffff"
    BG_ACCENT = "#e3f2fd"
    
    TEXT_PRIMARY = "#212121"
    TEXT_SECONDARY = "#757575"
    TEXT_SUCCESS = "#2e7d32"
    TEXT_ERROR = "#c62828"
    TEXT_WARNING = "#f57c00"
    TEXT_INFO = "#1565c0"
    
    FONT_TITLE = ("Microsoft YaHei UI", 14, "bold")
    FONT_NORMAL = ("Microsoft YaHei UI", 10)
    FONT_SMALL = ("Microsoft YaHei UI", 9)
    FONT_MONO = ("Consolas", 11)
    FONT_MONO_LARGE = ("Consolas", 14, "bold")
    
    PADDING = 10


# ==================== UI组件 ====================
class StatusIndicator(tk.Canvas):
    COLORS = {
        "idle": "#9e9e9e",
        "connecting": "#ffa000",
        "connected": "#4caf50",
        "running": "#2196f3",
        "error": "#f44336",
        "stopped": "#ff5722"
    }
    
    def __init__(self, parent, size: int = 12, **kwargs):
        super().__init__(parent, width=size, height=size, 
                        highlightthickness=0, bg=parent.cget('bg'), **kwargs)
        self.size = size
        self.status = "idle"
        self._draw()
    
    def _draw(self):
        self.delete("all")
        color = self.COLORS.get(self.status, self.COLORS["idle"])
        padding = 2
        self.create_oval(padding, padding, self.size - padding, self.size - padding,
                        fill=color, outline="")
    
    def set_status(self, status: str):
        self.status = status
        self._draw()


class LogPanel(tk.Frame):
    TAG_COLORS = {
        "DEBUG": "#607d8b",
        "INFO": "#1976d2",
        "WARNING": "#ff9800",
        "ERROR": "#f44336",
        "SUCCESS": "#4caf50"
    }
    
    def __init__(self, parent, height: int = 8, **kwargs):
        super().__init__(parent, **kwargs)
        
        header = tk.Frame(self, bg=ThemeConfig.BG_ACCENT)
        header.pack(fill=tk.X)
        
        tk.Label(header, text="📋 调试日志", font=ThemeConfig.FONT_NORMAL,
                bg=ThemeConfig.BG_ACCENT, fg=ThemeConfig.TEXT_PRIMARY).pack(side=tk.LEFT, padx=10, pady=5)
        
        tk.Button(header, text="清除", font=ThemeConfig.FONT_SMALL,
                 command=self.clear, relief=tk.FLAT, bg=ThemeConfig.BG_ACCENT).pack(side=tk.RIGHT, padx=5)
        
        text_frame = tk.Frame(self)
        text_frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        self.text = tk.Text(text_frame, height=height, wrap=tk.WORD,
                           font=ThemeConfig.FONT_MONO, bg="#fafafa",
                           relief=tk.FLAT, state=tk.DISABLED)
        
        scrollbar = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=self.text.yview)
        self.text.configure(yscrollcommand=scrollbar.set)
        
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        for tag, color in self.TAG_COLORS.items():
            self.text.tag_configure(tag, foreground=color)
    
    def log(self, message: str, level: str = "INFO"):
        self.text.configure(state=tk.NORMAL)
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.text.insert(tk.END, f"[{timestamp}] ", "DEBUG")
        self.text.insert(tk.END, f"[{level}] ", level)
        self.text.insert(tk.END, f"{message}\n")
        self.text.see(tk.END)
        self.text.configure(state=tk.DISABLED)
    
    def clear(self):
        self.text.configure(state=tk.NORMAL)
        self.text.delete(1.0, tk.END)
        self.text.configure(state=tk.DISABLED)


class LabeledEntry(tk.Frame):
    def __init__(self, parent, label: str, default: str = "", width: int = 15, **kwargs):
        super().__init__(parent, bg=ThemeConfig.BG_SECONDARY, **kwargs)
        
        tk.Label(self, text=label, font=ThemeConfig.FONT_NORMAL,
                bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_PRIMARY, anchor="w").pack(fill=tk.X)
        
        self.entry = ttk.Entry(self, width=width, font=ThemeConfig.FONT_NORMAL)
        self.entry.insert(0, default)
        self.entry.pack(fill=tk.X, pady=(2, 0))
    
    def get(self) -> str:
        return self.entry.get()
    
    def set(self, value: str):
        self.entry.delete(0, tk.END)
        self.entry.insert(0, value)


class SectionFrame(tk.LabelFrame):
    def __init__(self, parent, title: str, **kwargs):
        super().__init__(parent, text=f"  {title}  ", font=ThemeConfig.FONT_NORMAL,
                        bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_PRIMARY,
                        relief=tk.GROOVE, **kwargs)


# ==================== 预测结果显示组件 ====================
class PredictionDisplay(tk.Frame):
    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=ThemeConfig.BG_SECONDARY, **kwargs)
        
        phase_section = tk.Frame(self, bg=ThemeConfig.BG_SECONDARY)
        phase_section.pack(fill=tk.X, pady=(0, 10))
        
        tk.Label(phase_section, text="步态相位", font=ThemeConfig.FONT_SMALL,
                bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_SECONDARY).pack(anchor="w")
        
        self.phase_label = tk.Label(phase_section, text="--", 
                                    font=("Consolas", 28, "bold"),
                                    bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_INFO)
        self.phase_label.pack(anchor="w")
        
        self.phase_progress = ttk.Progressbar(phase_section, length=200, mode='determinate')
        self.phase_progress.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Separator(self, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        knee_frame = tk.Frame(self, bg=ThemeConfig.BG_SECONDARY)
        knee_frame.pack(fill=tk.X)
        
        left_section = tk.Frame(knee_frame, bg=ThemeConfig.BG_SECONDARY)
        left_section.pack(side=tk.LEFT, expand=True, fill=tk.X)
        
        tk.Label(left_section, text="左膝角度", font=ThemeConfig.FONT_SMALL,
                bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_SECONDARY).pack(anchor="w")
        self.left_knee_label = tk.Label(left_section, text="--°", 
                                        font=ThemeConfig.FONT_MONO_LARGE,
                                        bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_SUCCESS)
        self.left_knee_label.pack(anchor="w")
        
        right_section = tk.Frame(knee_frame, bg=ThemeConfig.BG_SECONDARY)
        right_section.pack(side=tk.LEFT, expand=True, fill=tk.X)
        
        tk.Label(right_section, text="右膝角度", font=ThemeConfig.FONT_SMALL,
                bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_SECONDARY).pack(anchor="w")
        self.right_knee_label = tk.Label(right_section, text="--°", 
                                         font=ThemeConfig.FONT_MONO_LARGE,
                                         bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_WARNING)
        self.right_knee_label.pack(anchor="w")
        
        ttk.Separator(self, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        status_frame = tk.Frame(self, bg=ThemeConfig.BG_SECONDARY)
        status_frame.pack(fill=tk.X)
        
        tk.Label(status_frame, text="状态:", font=ThemeConfig.FONT_SMALL,
                bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_SECONDARY).pack(side=tk.LEFT)
        self.status_label = tk.Label(status_frame, text="等待启动", font=ThemeConfig.FONT_SMALL,
                                     bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_SECONDARY)
        self.status_label.pack(side=tk.LEFT, padx=5)
        
        tk.Label(status_frame, text="FPS:", font=ThemeConfig.FONT_SMALL,
                bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_SECONDARY).pack(side=tk.LEFT, padx=(20, 0))
        self.fps_label = tk.Label(status_frame, text="--", font=ThemeConfig.FONT_SMALL,
                                  bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_INFO)
        self.fps_label.pack(side=tk.LEFT, padx=5)
    
    def update_prediction(self, result: Dict[str, float]):
        phase = result.get('gait_phase', 0)
        left = result.get('left_knee_angle', 0)
        right = result.get('right_knee_angle', 0)
        
        self.phase_label.config(text=f"{phase:.3f}")
        self.phase_progress['value'] = phase * 100
        self.left_knee_label.config(text=f"{left:.1f}°")
        self.right_knee_label.config(text=f"{right:.1f}°")
    
    def update_status(self, status: str, fps: float = 0):
        color_map = {
            "running": ThemeConfig.TEXT_SUCCESS,
            "stopped": ThemeConfig.TEXT_WARNING,
            "error": ThemeConfig.TEXT_ERROR,
            "waiting": ThemeConfig.TEXT_SECONDARY
        }
        self.status_label.config(text=status, fg=color_map.get(status, ThemeConfig.TEXT_SECONDARY))
        self.fps_label.config(text=f"{fps:.1f}" if fps > 0 else "--")
    
    def reset(self):
        self.phase_label.config(text="--")
        self.phase_progress['value'] = 0
        self.left_knee_label.config(text="--°")
        self.right_knee_label.config(text="--°")
        self.status_label.config(text="等待启动", fg=ThemeConfig.TEXT_SECONDARY)
        self.fps_label.config(text="--")


# ==================== 主UI类 ====================
class SensorUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("步态分析系统 - 传感器读取与推理")
        self.root.configure(bg=ThemeConfig.BG_PRIMARY)
        self.root.minsize(800, 700)
        
        self.logger = setup_logging(
            log_file=f"sensor_ui_{datetime.now().strftime('%Y%m%d')}.log"
        )
        self.logger.info("界面初始化开始")
        
        self.cache: Dict[str, Any] = {"imu": {}, "joint": {}, "dh": None, "fk": None}
        self._cache_lock = threading.Lock()
        
        self.receiver = None
        self.btmanager = None
        self.kinematic_model = None
        self.json_file_path: Optional[str] = None
        
        self.inference_thread: Optional[InferenceThread] = None
        self._latest_prediction: Optional[Dict[str, float]] = None
        self._prediction_lock = threading.Lock()
        
        self._is_connected = False
        self._is_receiving = False
        self._is_predicting = False
        self._ui_update_timer = None
        
        self._build_ui()
        self._apply_styles()
        
        self.logger.info("界面初始化完成")
        self.log_panel.log("系统就绪，请选择配置文件并初始化", "SUCCESS")
    
    def _apply_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TButton", font=ThemeConfig.FONT_NORMAL, padding=6)
        style.configure("TEntry", font=ThemeConfig.FONT_NORMAL, padding=4)
        style.configure("TProgressbar", thickness=8)
    
    def _build_ui(self):
        main_container = tk.Frame(self.root, bg=ThemeConfig.BG_PRIMARY)
        main_container.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)
        
        self._build_title_bar(main_container)
        
        content_frame = tk.Frame(main_container, bg=ThemeConfig.BG_PRIMARY)
        content_frame.pack(fill=tk.BOTH, expand=True, pady=(15, 0))
        
        left_panel = tk.Frame(content_frame, bg=ThemeConfig.BG_PRIMARY, width=320)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left_panel.pack_propagate(False)
        
        self._build_config_section(left_panel)
        self._build_control_section(left_panel)
        
        right_panel = tk.Frame(content_frame, bg=ThemeConfig.BG_PRIMARY)
        right_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self._build_prediction_section(right_panel)
        self._build_log_section(right_panel)
    
    def _build_title_bar(self, parent):
        title_frame = tk.Frame(parent, bg=ThemeConfig.BG_PRIMARY)
        title_frame.pack(fill=tk.X)
        
        tk.Label(title_frame, text="🦿 步态分析系统", 
                font=ThemeConfig.FONT_TITLE, bg=ThemeConfig.BG_PRIMARY,
                fg=ThemeConfig.TEXT_PRIMARY).pack(side=tk.LEFT)
        
        status_frame = tk.Frame(title_frame, bg=ThemeConfig.BG_PRIMARY)
        status_frame.pack(side=tk.RIGHT)
        
        self.status_indicator = StatusIndicator(status_frame)
        self.status_indicator.pack(side=tk.LEFT, padx=(0, 5))
        
        self.status_label = tk.Label(status_frame, text="未启动",
                                    font=ThemeConfig.FONT_NORMAL,
                                    bg=ThemeConfig.BG_PRIMARY, fg=ThemeConfig.TEXT_SECONDARY)
        self.status_label.pack(side=tk.LEFT)
    
    def _build_config_section(self, parent):
        config_frame = SectionFrame(parent, "参数配置")
        config_frame.pack(fill=tk.X, pady=(0, 10))
        
        inner = tk.Frame(config_frame, bg=ThemeConfig.BG_SECONDARY)
        inner.pack(fill=tk.X, padx=10, pady=10)
        
        self.port_entry = LabeledEntry(inner, "串口名称:", "COM19")
        self.port_entry.pack(fill=tk.X, pady=3)
        
        tk.Label(inner, text="运动学参数 (米)", font=ThemeConfig.FONT_SMALL,
                bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_SECONDARY).pack(anchor="w", pady=(10, 5))
        
        params_frame = tk.Frame(inner, bg=ThemeConfig.BG_SECONDARY)
        params_frame.pack(fill=tk.X)
        
        self.thigh_entry = LabeledEntry(params_frame, "大腿:", "0.4", width=8)
        self.thigh_entry.pack(side=tk.LEFT, padx=(0, 5))
        
        self.shank_entry = LabeledEntry(params_frame, "小腿:", "0.43", width=8)
        self.shank_entry.pack(side=tk.LEFT, padx=(0, 5))
        
        self.hip_entry = LabeledEntry(params_frame, "髋宽:", "0.2", width=8)
        self.hip_entry.pack(side=tk.LEFT)
        
        tk.Label(inner, text="推理配置文件:", font=ThemeConfig.FONT_NORMAL,
                bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_PRIMARY).pack(anchor="w", pady=(10, 2))
        
        json_frame = tk.Frame(inner, bg=ThemeConfig.BG_SECONDARY)
        json_frame.pack(fill=tk.X)
        
        self.json_path_var = tk.StringVar(value="未选择")
        tk.Label(json_frame, textvariable=self.json_path_var, font=ThemeConfig.FONT_SMALL,
                bg=ThemeConfig.BG_SECONDARY, fg=ThemeConfig.TEXT_INFO, anchor="w").pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        ttk.Button(json_frame, text="浏览...", width=8, command=self.select_json_file).pack(side=tk.RIGHT)
    
    def _build_control_section(self, parent):
        control_frame = SectionFrame(parent, "操作控制")
        control_frame.pack(fill=tk.X)
        
        inner = tk.Frame(control_frame, bg=ThemeConfig.BG_SECONDARY)
        inner.pack(fill=tk.X, padx=10, pady=10)
        
        self.btn_init = ttk.Button(inner, text="🔄 初始化模型", command=self.init_model, width=25)
        self.btn_init.pack(fill=tk.X, pady=3)
        
        imu_frame = tk.Frame(inner, bg=ThemeConfig.BG_SECONDARY)
        imu_frame.pack(fill=tk.X, pady=3)
        
        self.btn_connect_imu = ttk.Button(imu_frame, text="📡 连接IMU", command=self.connect_imu, width=12)
        self.btn_connect_imu.pack(side=tk.LEFT, padx=(0, 5))
        
        self.btn_disconnect_imu = ttk.Button(imu_frame, text="断开IMU", command=self.disconnect_imu, width=12, state=tk.DISABLED)
        self.btn_disconnect_imu.pack(side=tk.LEFT)
        
        recv_frame = tk.Frame(inner, bg=ThemeConfig.BG_SECONDARY)
        recv_frame.pack(fill=tk.X, pady=3)
        
        self.btn_start_recv = ttk.Button(recv_frame, text="▶ 开始接收", command=self.start_receiver, width=12)
        self.btn_start_recv.pack(side=tk.LEFT, padx=(0, 5))
        
        self.btn_stop_recv = ttk.Button(recv_frame, text="⏹ 停止接收", command=self.stop_receiver, width=12, state=tk.DISABLED)
        self.btn_stop_recv.pack(side=tk.LEFT)
        
        ttk.Separator(inner, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        pred_frame = tk.Frame(inner, bg=ThemeConfig.BG_SECONDARY)
        pred_frame.pack(fill=tk.X, pady=3)
        
        self.btn_start_predict = ttk.Button(pred_frame, text="🎯 开始推理", command=self.start_prediction, width=12)
        self.btn_start_predict.pack(side=tk.LEFT, padx=(0, 5))
        
        self.btn_stop_predict = ttk.Button(pred_frame, text="停止推理", command=self.stop_prediction, width=12, state=tk.DISABLED)
        self.btn_stop_predict.pack(side=tk.LEFT)
        
        ttk.Separator(inner, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        self.btn_simulate = ttk.Button(inner, text="🧪 模拟数据测试", command=self.start_simulation, width=25)
        self.btn_simulate.pack(fill=tk.X, pady=3)
    
    def _build_prediction_section(self, parent):
        pred_frame = SectionFrame(parent, "实时预测结果")
        pred_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.prediction_display = PredictionDisplay(pred_frame)
        self.prediction_display.pack(fill=tk.X, padx=10, pady=10)
    
    def _build_log_section(self, parent):
        self.log_panel = LogPanel(parent, height=12)
        self.log_panel.pack(fill=tk.BOTH, expand=True)
    
    # ==================== 状态管理 ====================
    
    def _update_status(self, status: str, message: str):
        self.status_indicator.set_status(status)
        self.status_label.config(text=message)
        
        color_map = {
            "idle": ThemeConfig.TEXT_SECONDARY,
            "connecting": ThemeConfig.TEXT_WARNING,
            "connected": ThemeConfig.TEXT_SUCCESS,
            "running": ThemeConfig.TEXT_INFO,
            "error": ThemeConfig.TEXT_ERROR,
            "stopped": ThemeConfig.TEXT_ERROR
        }
        self.status_label.config(fg=color_map.get(status, ThemeConfig.TEXT_SECONDARY))
    
    # ==================== 推理相关 ====================
    
    def _init_inference(self) -> bool:
        if not self.json_file_path:
            self.log_panel.log("请先选择JSON配置文件", "WARNING")
            messagebox.showwarning("提示", "请先选择JSON配置文件")
            return False
        
        try:
            if self.inference_thread:
                self.inference_thread.stop()
                self.inference_thread = None
            
            self.inference_thread = InferenceThread(
                config_path=self.json_file_path,
                freq=1000,
                device="auto"
            )
            self.inference_thread.set_callback(self._on_prediction_result)
            
            # 同步 UI 已有的 cache 数据到推理器（带名称映射）
            with self._cache_lock:
                # IMU 数据同步（带名称映射）
                name_map = {"left": "left_thigh", "right": "right_thigh"}
                imu_to_sync = {}
                for dev_id, data in self.cache.get("imu", {}).items():
                    mapped_id = name_map.get(dev_id, dev_id)
                    imu_to_sync[mapped_id] = data
                
                joint_to_sync = self.cache.get("joint", {}).copy()
                
                if imu_to_sync or joint_to_sync:
                    self.inference_thread.update(imu=imu_to_sync, joint=joint_to_sync)
                    self.log_panel.log(
                        f"已同步缓存: IMU={list(imu_to_sync.keys())}, Joint={list(joint_to_sync.keys())}", 
                        "INFO"
                    )
            
            info = self.inference_thread.info()
            self.log_panel.log(f"推理器初始化: {info['inferencer']['model_type']}", "SUCCESS")
            self.logger.info(f"推理器已初始化: {info}")
            return True
            
        except Exception as e:
            self.log_panel.log(f"推理器初始化失败: {e}", "ERROR")
            self.logger.error(f"推理器初始化失败: {e}", exc_info=True)
            messagebox.showerror("初始化失败", str(e))
            return False
    
    def _on_prediction_result(self, result: Dict[str, float]):
        with self._prediction_lock:
            self._latest_prediction = result
        print(result)
        # 发送到MCU（如果有receiver）
        if self.receiver:
            try:
                phase = result.get('gait_phase', 0.0)
                left = result.get('left_knee_angle', 0.0)
                right = result.get('right_knee_angle', 0.0)
                data_str = f"{phase}, {left}, {right}"
                self.receiver.send_line(data_str)
            except Exception as e:
                # self.logger.error(f"发送推理结果失败: {e}")
                pass
        
        self.root.after(0, self._update_prediction_ui)
    
    def _update_prediction_ui(self):
        with self._prediction_lock:
            result = self._latest_prediction
        
        if result and self.inference_thread:
            self.prediction_display.update_prediction(result)
            fps = self.inference_thread.get_stats().get('last_fps', 0)
            self.prediction_display.update_status("running", fps)
    
    # ==================== 数据回调 ====================
    
    def on_imu(self, data:btc.IMUSensorData):
        """IMU数据回调 - 从传感器模块调用"""
        try:
            # 名称映射（兼容旧命名）
            device_id = data.device_id
            name_map = {"left": "left_thigh", "right": "right_thigh"}
            device_id = name_map.get(device_id, device_id)
            
            imu_data = imu_adp.sensor_to_imu(data)
            
            with self._cache_lock:
                self.cache["imu"][device_id] = imu_data
            
            # 只要推理线程存在就更新，不管是否正在推理
            # 这样推理器的 cache 始终保持最新
            if self.inference_thread:
                self.inference_thread.update_imu(device_id, imu_data)
        except Exception as e:
            self.logger.error(f"处理IMU数据失败: {e}")
    
    def on_joint(self, timestamp , joints):
        """关节数据回调 - 从传感器模块调用"""
        try:
            joint_data = {}
            if len(joints) > 0:
                joint_data["right_knee"] = enc_adp.joint_dict_to_encoder(joints[0])
            if len(joints) > 1:
                joint_data["left_knee"] = enc_adp.joint_dict_to_encoder(joints[1])
            
            with self._cache_lock:
                self.cache["joint"].update(joint_data)

            # 只要推理线程存在就更新，不管是否正在推理
            if self.inference_thread:
                for joint_id, data in joint_data.items():
                    self.inference_thread.update_joint(joint_id, data)
                    
        except Exception as e:
            self.logger.error(f"处理关节数据失败: {e}")
    
    # ==================== 操作函数 ====================
    
    def select_json_file(self):
        file_path = filedialog.askopenfilename(
            title="选择推理配置文件",
            filetypes=[("JSON 文件", "*.json"), ("所有文件", "*.*")]
        )
        if file_path:
            self.json_file_path = file_path
            self.json_path_var.set(os.path.basename(file_path))
            self.log_panel.log(f"已选择配置: {os.path.basename(file_path)}", "INFO")
    
    def init_model(self):
        self.log_panel.log("开始初始化...", "INFO")
        
        try:
            # TODO: 添加你的实际初始化代码

            self.btmanager = btc.MultiIMUManager()
            self.btmanager.add_device("pelvis", "00:04:3E:6C:51:C1")
            self.btmanager.add_device("right_thigh", "00:04:3E:86:27:F0")  # 修复：改为 right_thigh
            self.btmanager.add_device("left_thigh", "00:04:3E:86:27:ED")   # 修复：改为 left_thigh
            self.btmanager.register_callback(self.on_imu)

            self.receiver = m2p.SerialReceiver(port = self.port_entry.get(), 
                                            baudrate=460800, output_csv="./joint.csv")
            self.receiver.set_data_callback(self.on_joint)
            
            self._update_status("idle", "模型已初始化")
            self.log_panel.log("模型初始化成功 ✔", "SUCCESS")
            
        except Exception as e:
            self.log_panel.log(f"初始化失败: {e}", "ERROR")
            messagebox.showerror("初始化失败", str(e))
    
    def connect_imu(self):
        self.log_panel.log("连接IMU...", "INFO")
        self._update_status("connecting", "正在连接...")
        
        # TODO: 添加实际连接代码
        connection_results = self.btmanager.connect_all(timeout=10.0, parallel=True)
        connected_devices = self.btmanager.get_connected_devices()
        self.root.after(1000, self._on_imu_connected, ["pelvis", "left_thigh", "right_thigh"])
    
    def _on_imu_connected(self, devices):
        self._is_connected = True
        self.btn_connect_imu.config(state=tk.DISABLED)
        self.btn_disconnect_imu.config(state=tk.NORMAL)
        self._update_status("connected", f"已连接 {len(devices)} 设备")
        self.log_panel.log(f"IMU连接成功: {', '.join(devices)}", "SUCCESS")
    
    def disconnect_imu(self):
        self._is_connected = False
        self.btn_disconnect_imu.config(state=tk.DISABLED)
        self.btn_connect_imu.config(state=tk.NORMAL)
        self._update_status("idle", "已断开")
        self.log_panel.log("IMU已断开", "INFO")
    
    def start_receiver(self):
        self._is_receiving = True
        self.btn_start_recv.config(state=tk.DISABLED)
        self.btn_stop_recv.config(state=tk.NORMAL)
        self._update_status("running", "接收数据中...")
        self.log_panel.log("开始接收数据", "SUCCESS")
        
        # TODO: 启动实际数据接收
        self.receiver.start()
        self.btmanager.start_all()
    
    def stop_receiver(self):
        self._is_receiving = False
        if self.btmanager:
            self.btmanager.stop_all()
        if self.receiver:
            self.receiver.stop()
        self.btn_start_recv.config(state=tk.NORMAL)
        self.btn_stop_recv.config(state=tk.DISABLED)
        self._update_status("stopped", "已停止")
        self.log_panel.log("数据接收已停止", "WARNING")
    
    def start_prediction(self):
        # 每次启动都重新初始化推理器，确保状态干净
        if not self._init_inference():
            return
            
        print("Starting prediction...")
        try:
            self._is_predicting = True
            self.inference_thread.start()
            
            self.btn_start_predict.config(state=tk.DISABLED)
            self.btn_stop_predict.config(state=tk.NORMAL)
            
            self.prediction_display.update_status("running", 0)
            self.log_panel.log("步态推理已启动", "SUCCESS")
            
        except Exception as e:
            self._is_predicting = False
            self.log_panel.log(f"启动推理失败: {e}", "ERROR")
    
    def stop_prediction(self):
        self._is_predicting = False
        
        if self.inference_thread:
            stats = self.inference_thread.get_stats()
            self.inference_thread.stop()
            self.inference_thread = None  # 清除，下次启动重新创建
            
            self.log_panel.log(
                f"推理统计: 帧数={stats['frame_count']}, FPS={stats['last_fps']:.1f}",
                "INFO"
            )
        
        self.btn_stop_predict.config(state=tk.DISABLED)
        self.btn_start_predict.config(state=tk.NORMAL)
        
        self.prediction_display.update_status("stopped", 0)
        self.log_panel.log("步态推理已停止", "INFO")
    
    # ==================== 模拟测试 ====================
    
    def start_simulation(self):
        if not self._init_inference():
            return
        
        self.log_panel.log("启动模拟数据测试...", "INFO")
        
        self._is_predicting = True
        self.inference_thread.start()
        
        self.btn_start_predict.config(state=tk.DISABLED)
        self.btn_stop_predict.config(state=tk.NORMAL)
        self.btn_simulate.config(state=tk.DISABLED)
        
        self._simulation_frame = 0
        self._simulate_data()
    
    def _simulate_data(self):
        if not self._is_predicting:
            self.btn_simulate.config(state=tk.NORMAL)
            return
        
        t = self._simulation_frame * 0.01
        self._simulation_frame += 1
        
        imu_data = {
            "pelvis": IMUData(
                accelerometer=np.array([0.1 * np.sin(t), 9.8, 0.1 * np.cos(t)]),
                gyroscope=np.array([0.01, 0.02, 0.03]),
                quaternion=np.array([1.0, 0.0, 0.0, 0.0])
            ),
            "left_thigh": IMUData(
                accelerometer=np.array([0.2 * np.sin(t + 1), 9.7, 0.2 * np.cos(t + 1)]),
                gyroscope=np.array([0.02, 0.03, 0.04]),
                quaternion=np.array([0.99, 0.1, 0.0, 0.0])
            ),
            "right_thigh": IMUData(
                accelerometer=np.array([0.2 * np.sin(t + 2), 9.7, 0.2 * np.cos(t + 2)]),
                gyroscope=np.array([0.02, 0.03, 0.04]),
                quaternion=np.array([0.99, -0.1, 0.0, 0.0])
            )
        }
        
        joint_data = {
            "left_knee": EncoderData(
                angle=25.0 + 10 * np.sin(t * 2),
                velocity=1.2,
                t_ff=0.5
            ),
            "right_knee": EncoderData(
                angle=25.0 + 10 * np.sin(t * 2 + np.pi),
                velocity=1.5,
                t_ff=0.6
            )
        }
        
        self.inference_thread.update(imu=imu_data, joint=joint_data)
        
        self._ui_update_timer = self.root.after(10, self._simulate_data)
    
    # ==================== 资源清理 ====================
    
    def get_latest_prediction(self) -> Optional[Dict[str, float]]:
        with self._prediction_lock:
            return copy.deepcopy(self._latest_prediction) if self._latest_prediction else None
    
    def on_closing(self):
        self.logger.info("正在关闭应用...")
        
        if self._ui_update_timer:
            self.root.after_cancel(self._ui_update_timer)
        
        if self._is_predicting:
            self.stop_prediction()
        
        if self._is_receiving:
            self.stop_receiver()
        if self._is_connected:
            self.disconnect_imu()
        
        if self.inference_thread:
            self.inference_thread.stop()
            self.inference_thread = None
        
        self.logger.info("应用已关闭")
        self.root.destroy()


# ==================== 程序入口 ====================
def main():
    root = tk.Tk()
    
    window_width, window_height = 850, 700
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    app = SensorUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()