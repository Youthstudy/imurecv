import threading
import time
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
import os
import sensorread.MCU2PC as m2p
import sensorread.bt_connect as btc
import sensorread.kinematic_model_core as km
import sensorread.imu_adapter as imu_adapter
import sensorread.encoder_adapter as encoder_adapter


# ==================== UI 部分 ====================
class UI:
    def __init__(self, root):
        self.root = root
        self.root.title("串口接收控制")

        # 数据缓存
        self.cache = {}

        # 创建主框架
        main_frame = tk.Frame(root)
        main_frame.pack(pady=20, padx=20)
        
        # 串口配置
        tk.Label(main_frame, text="串口名称:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.port_entry = tk.Entry(main_frame, width=15)
        self.port_entry.insert(0, "COM19")
        self.port_entry.grid(row=0, column=1, padx=5, pady=5)

        # 大腿长度
        tk.Label(main_frame, text="大腿长度:").grid(row=1, column=0, sticky="w", padx=5, pady=5)
        self.port_km_thigh = tk.Entry(main_frame, width=15)
        self.port_km_thigh.insert(0, "0.4")
        self.port_km_thigh.grid(row=1, column=1, padx=5, pady=5)

        # 小腿长度
        tk.Label(main_frame, text="小腿长度:").grid(row=2, column=0, sticky="w", padx=5, pady=5)
        self.port_km_shank = tk.Entry(main_frame, width=15)
        self.port_km_shank.insert(0, "0.43")
        self.port_km_shank.grid(row=2, column=1, padx=5, pady=5)

        # 髋部宽度
        tk.Label(main_frame, text="髋部宽度:").grid(row=3, column=0, sticky="w", padx=5, pady=5)
        self.port_km_hip = tk.Entry(main_frame, width=15)
        self.port_km_hip.insert(0, "0.2")
        self.port_km_hip.grid(row=3, column=1, padx=5, pady=5)

        # 按钮框架
        button_frame = tk.Frame(main_frame)
        button_frame.grid(row=5, column=0, columnspan=2, pady=10)

        # JSON 文件选择
        self.json_label = tk.Label(main_frame, text="JSON 配置文件:")
        self.json_label.grid(row=4, column=0, sticky="w", padx=5, pady=5)
        self.json_path_var = tk.StringVar(value="未选择")
        self.json_path_label = tk.Label(main_frame, textvariable=self.json_path_var, fg="blue")
        self.json_path_label.grid(row=4, column=1, sticky="w", padx=5, pady=5)

        # 在 button_frame 中添加选择按钮
        self.select_json_button = tk.Button(button_frame, text="选择JSON", command=self.select_json_file)
        self.select_json_button.pack(side=tk.LEFT, padx=5)

        self.model_init = tk.Button(button_frame, text="初始化模型", command=self.Init_model)
        self.model_init.pack(side=tk.LEFT, padx=5)

        self.connect_imu_button = tk.Button(button_frame, text="连接imu", command=self.connect)
        self.connect_imu_button.pack(side=tk.LEFT, padx=5)

        self.disconnect_imu = tk.Button(button_frame, text="断开imu连接", command=self.disconnect_bt, state=tk.DISABLED)
        self.disconnect_imu.pack(side=tk.LEFT, padx=5)

        self.start_button = tk.Button(button_frame, text="开始接收", command=self.start_receiver)
        self.start_button.pack(side=tk.LEFT, padx=5)

        self.stop_button = tk.Button(button_frame, text="停止接收", command=self.stop_receiver, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=5)

        self.status_label = tk.Label(main_frame, text="状态: 未启动", fg="gray")
        self.status_label.grid(row=6, column=0, columnspan=2, pady=10)

        self.model_predict_start_button = tk.Button(button_frame, text="开始预测", command=self.mode_predict_start)
        self.model_predict_start_button.pack(side=tk.LEFT, padx=5)

        self.model_predict_stop_button = tk.Button(button_frame, text="停止预测", command=self.mode_predict_stop, state=tk.DISABLED)
        self.model_predict_stop_button.pack(side=tk.LEFT, padx=5)

        self.receiver = None
        self.btmanager = None
        self.p2m_sender = None
        self.kinematic_model = None

    def on_imu(self, data):
        self.cache["imu"][data.device_id] = imu_adapter.sensor_to_imu(data)

    def on_joint(self, joints):
        if len(joints) > 0:
            self.cache["joint"]["right_knee"] = encoder_adapter.joint_dict_to_encoder(joints[0])
        if len(joints) > 1:
            self.cache["joint"]["left_knee"] = encoder_adapter.joint_dict_to_encoder(joints[1])

    def select_json_file(self):
        file_path = filedialog.askopenfilename(
            title="选择配置文件",
            filetypes=[("JSON 文件", "*.json"), ("所有文件", "*.*")]
        )
        if file_path:
            self.json_file_path = file_path  # 保存路径供后续使用
            filename = os.path.basename(file_path)
            self.json_path_var.set(filename)
            # 可选：打印或加载 JSON 内容
            print(f"已选择 JSON 文件: {file_path}")

    def kinematic_update(self):
        imu = self.cache.get("imu", {})
        joint = self.cache.get("joint", {})
        
        if all(k in imu for k in ["pelvis", "left", "right"]) and \
        all(k in joint for k in ["left_knee", "right_knee"]):
            
            feat = self.kinematic_model.update(
                imu["pelvis"],
                imu["right"],
                imu["left"],
                joint["right_knee"],
                joint["left_knee"]
            )
            self.cache["dh"] = feat["dh"]
            self.cache["fk"] = feat["fk"]

    def Init_model(self):
        self.btmanager = btc.MultiIMUManager()
        self.btmanager.add_device("pelvis", "00:04:3E:6C:51:C1")
        self.btmanager.add_device("right", "00:04:3E:86:27:F0")  # 替换为实际MAC地址
        self.btmanager.add_device("left", "00:04:3E:86:27:ED") 
        self.btmanager.register_callback(self.on_imu)

        self.receiver = m2p.SerialReceiver(port = self.port_entry.get(), 
                                           baudrate=460800, output_csv="./joint.csv")
        self.receiver.set_data_callback(self.on_joint)

        self.kinematic_model = km.KinematicModel(
            thigh_length=float(self.port_km_thigh.get()),
            shank_length=float(self.port_km_shank.get()),
            hip_width=float(self.port_km_hip.get())
        )

    def connect(self):
        try:
            connection_results = self.btmanager.connect_all(timeout=10.0, parallel=True)
            connected_devices = self.btmanager.get_connected_devices()
            print(f"\n已连接的设备: {', '.join(connected_devices)}")

            self.connect_imu_button.config(state=tk.DISABLED)
            self.disconnect_imu.config(state=tk.NORMAL)
            
        except Exception as e:
            messagebox.showerror("错误", f"连接失败：{e}")

    def disconnect_bt(self):
        if self.btmanager:
            self.btmanager.disconnect_all()
            self.disconnect_imu.config(state=tk.DISABLED)
            self.connect_imu_button.config(state=tk.NORMAL)

    def start_receiver(self):
        try:
            self.receiver.start()
            self.btmanager.start_all()
            self.status_label.config(text=f"状态: 运行中（保存到 {os.path.basename(self.receiver.output_csv)}）", fg="green")
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.NORMAL)
        except Exception as e:
            messagebox.showerror("错误", f"启动失败：{e}")

    def stop_receiver(self):
        if self.btmanager:
            self.btmanager.stop_all()
        if self.receiver:
            self.receiver.stop()
            self.status_label.config(text="状态: 已停止", fg="red")
            self.start_button.config(state=tk.NORMAL)
            self.stop_button.config(state=tk.DISABLED)
        
    def mode_predict_start(self):

        self.model_predict_stop_button.config(state=tk.NORMAL)
        self.model_predict_start_button.config(state=tk.DISABLED)

    def mode_predict_stop(self):

        self.model_predict_stop_button.config(state=tk.DISABLED)
        self.model_predict_start_button.config(state=tk.NORMAL)

if __name__ == "__main__":
    root = tk.Tk()
    root.title("操作界面")
    root.geometry("500x400")
    app = UI(root)
    root.mainloop()



