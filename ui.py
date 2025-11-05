import threading
import time
import tkinter as tk
from tkinter import messagebox
import os
import MCU2PC as m2p
import bt_connect as btc

# ==================== UI 部分 ====================
class UI:
    def __init__(self, root):
        self.root = root
        self.root.title("串口接收控制")

        self.port_entry = tk.Entry(root)
        self.port_entry.insert(0, "COM19")
        self.port_entry.pack(pady=5)

        self.connect_imu_button = tk.Button(root, text="连接imu", command=self.connect)
        self.connect_imu_button.pack(pady=5)

        self.disconnect_imu = tk.Button(root, text="断开imu连接", command=self.disconnect_bt)
        self.disconnect_imu.pack(pady=5)

        self.start_button = tk.Button(root, text="开始接收", command=self.start_receiver)
        self.start_button.pack(pady=5)

        self.stop_button = tk.Button(root, text="停止接收", command=self.stop_receiver, state=tk.DISABLED)
        self.stop_button.pack(pady=5)

        self.status_label = tk.Label(root, text="状态: 未启动", fg="gray")
        self.status_label.pack(pady=5)

        self.receiver = None
        self.btmanager = None

    def connect(self):
        try:
            self.btmanager = btc.MultiIMUManager()
            self.btmanager.add_device("IMU_1", "00:04:3E:6C:51:C1")
            self.btmanager.add_device("IMU_2", "00:04:3E:86:27:F0")  # 替换为实际MAC地址
            self.btmanager.add_device("IMU_3", "00:04:3E:86:27:ED") 
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
        port = self.port_entry.get()
        try:
            self.receiver = m2p.SerialReceiver(port=port, baudrate=460800, output_csv="./joint.csv")
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
            self.btmanager.stop_all()
            self.status_label.config(text="状态: 已停止", fg="red")
            self.start_button.config(state=tk.NORMAL)
            self.stop_button.config(state=tk.DISABLED)

if __name__ == "__main__":
    root = tk.Tk()
    app = UI(root)
    root.mainloop()

