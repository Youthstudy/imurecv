import serial
import struct
import threading
import csv
import time
from datetime import datetime
import os

JOINT_SIZE = 36

def unpack_joint(payload):
    if len(payload) != JOINT_SIZE:
        raise ValueError(f"单个 joint 数据应为 {JOINT_SIZE} 字节，实际为 {len(payload)} 字节")

    ret0, ret1, ret2, p_des, v_des, kp, kd, t_ff, mode = struct.unpack('<8fI', payload)

    return {
        "knee_angle": ret0,
        "v": ret1,
        "t": ret2,
        "p_des": p_des,
        "v_des": v_des,
        "kp": kp,
        "kd": kd,
        "t_ff": t_ff,
        "mode": mode
    }

def unpack_frame(payload):
    joint_size = JOINT_SIZE
    joints = []
    for i in range(0, len(payload), joint_size):
        block = payload[i:i+joint_size]
        if len(block) == joint_size:
            joints.append(unpack_joint(block))  
    return joints

def get_unique_filename(path: str) -> str:
    base, ext = os.path.splitext(path)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{base}_{timestamp}{ext}"

class SerialReceiver:
    def __init__(self, port, baudrate=115200, output_csv="data_log.csv"):
        self.ser = serial.Serial(port, baudrate, timeout=0.05)
        self.buffer = bytearray()
        self.running = True
        self.output_csv = get_unique_filename(output_csv)

        self.data_callback = None

        self.csv_file = open(self.output_csv, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.shared_data = {
            "joint1": {
                "knee_angle": None,
                "v": None,
                "t": None,
                "p_des": None,
                "v_des": None,
                "kp": None,
                "kd": None,
                "t_ff": None,
                "mode": None
            },
            "joint2": {
                "knee_angle": None,
                "v": None,
                "t": None,
                "p_des": None,
                "v_des": None,
                "kp": None,
                "kd": None,
                "t_ff": None,
                "mode": None
            }
        }
        
        # 修改CSV表头，为两个关节并列存储
        headers = ["timestamp"]
        # 关节1的列
        joint1_headers = ["joint1_knee_angle", "joint1_v", "joint1_t", "joint1_p_des", 
                         "joint1_v_des", "joint1_kp", "joint1_kd", "joint1_t_ff", "joint1_mode"]
        # 关节2的列
        joint2_headers = ["joint2_knee_angle", "joint2_v", "joint2_t", "joint2_p_des", 
                         "joint2_v_des", "joint2_kp", "joint2_kd", "joint2_t_ff", "joint2_mode"]
        
        headers.extend(joint1_headers)
        headers.extend(joint2_headers)
        self.csv_writer.writerow(headers)

    def get_shared_data(self):
        """
        获取共享数据字典的副本
        """
        import copy
        return copy.deepcopy(self.shared_data)

    def set_data_callback(self, callback):
        self.data_callback = callback

    def send_line(self, data_str: str):
        """
        发送一行字符串（自动加 \\n）
        """
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial not opened")

        if not data_str.endswith("\n"):
            data_str += "\n"

        self.ser.write(data_str.encode("utf-8"))

    def send_floats(self, data, fmt="{:.4f}"):
        """
        发送浮点数组，例如 [1.2, 3.4, 5.6]
        -> "1.200,3.400,5.600\\n"
        """
        str_list = [fmt.format(x) for x in data]
        line = ",".join(str_list)
        self.send_line(line)

    def start(self):
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print(f"串口 {self.ser.port} 开始接收，保存位置到{self.output_csv}")

    def stop(self):
        self.running = False
        time.sleep(0.2)
        if self.ser.is_open:
            self.ser.close()
        self.csv_file.close()
        print("串口停止")

    def _read_loop(self):
        while self.running:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.buffer.extend(data)
                self._parse_buffer()

    def _parse_buffer(self):
        while True:
            if len(self.buffer) < 4:
                return

            try:
                start = self.buffer.index(0x3A)
            except ValueError:
                self.buffer.clear()
                return

            if start > 0:
                del self.buffer[:start]

            if len(self.buffer) < 2:
                return

            length = self.buffer[1]
            frame_len = 1 + 1 + length + 2

            if len(self.buffer) < frame_len:
                return 

            frame = self.buffer[:frame_len]

            if frame[-2:] != b'\x0D\x0A':
                del self.buffer[0]
                continue

            payload = frame[2:-2]
            self._handle_payload(payload)

            del self.buffer[:frame_len]

    def _handle_payload(self, payload):
        try:
            joints = unpack_frame(payload)
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            # 准备CSV行数据
            row_data = [timestamp]
            
            # 处理关节1数据（如果存在）
            if len(joints) > 0:
                j1 = joints[0]
                row_data.extend([
                    j1["knee_angle"], j1["v"], j1["t"],
                    j1["p_des"], j1["v_des"], j1["kp"], j1["kd"], j1["t_ff"], j1["mode"]
                ])

                self.shared_data["joint1"] = j1
            else:
                # 如果没有关节1数据，填入空值
                row_data.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0])
            
            # 处理关节2数据（如果存在）
            if len(joints) > 1:
                j2 = joints[1]
                row_data.extend([
                    j2["knee_angle"], j2["v"], j2["t"],
                    j2["p_des"], j2["v_des"], j2["kp"], j2["kd"], j2["t_ff"], j2["mode"]
                ])
                self.shared_data["joint2"] = j2
            else:
                # 如果没有关节2数据，填入空值
                row_data.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0])
            
            # 写入CSV
            self.csv_writer.writerow(row_data)
            self.csv_file.flush()
                            
            if self.data_callback:
                self.data_callback(timestamp, joints)
                
        except Exception as e:
            print(f"串口错误: {e}")


if __name__ == "__main__":
    receiver = SerialReceiver(port="COM19", baudrate=460800, output_csv="./joint.csv")
    receiver.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        receiver.stop()