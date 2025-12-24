import threading
import time
import tkinter as tk
from tkinter import messagebox
import os
import MCU2PC as m2p
import bt_connect as btc
import kinematic_model as km


# ==================== UI 部分 ====================
class UI:
    def __init__(self, root):
        self.root = root
        self.root.title("串口接收控制")

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

        self.receiver = None
        self.btmanager = None
        self.p2m_sender = None
        self.kinematic_model = None

    def Init_class(self):
        self.btmanager = btc.MultiIMUManager()
        self.btmanager.add_device("IMU_1", "00:04:3E:6C:51:C1")
        self.btmanager.add_device("IMU_2", "00:04:3E:86:27:F0")  # 替换为实际MAC地址
        self.btmanager.add_device("IMU_3", "00:04:3E:86:27:ED") 
        self.receiver = m2p.SerialReceiver(port = self.port_entry.get(), 
                                           baudrate=460800, output_csv="./joint.csv")
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
            
            # 清空IMU参数显示
            self.imu_param_display.config(state=tk.NORMAL)
            self.imu_param_display.delete(1.0, tk.END)
            self.imu_param_display.config(state=tk.DISABLED)

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
        if self.receiver:
            self.receiver.stop()
            self.status_label.config(text="状态: 已停止", fg="red")
            self.start_button.config(state=tk.NORMAL)
            self.stop_button.config(state=tk.DISABLED)

    def mode_predict_start(self):
        pass

    def mode_predict_stop(self):
        pass

if __name__ == "__main__":
    root = tk.Tk()
    root.title("操作界面")
    root.geometry("500x400")
    app = UI(root)
    root.mainloop()



