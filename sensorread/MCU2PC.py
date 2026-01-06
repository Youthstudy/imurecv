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

def get_unique_filename(filename: str) -> str:
    """为文件名添加时间戳"""
    base, ext = os.path.splitext(filename)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{base}_{timestamp}{ext}"

class SerialReceiver:
    def __init__(self, port, baudrate=115200, output_csv="data_log.csv", save_path=None):
        self.ser = serial.Serial(port, baudrate, timeout=0.05)
        self.buffer = bytearray()
        self.running = True
        self.data_callback = None
        self._csv_lock = threading.Lock()  # CSV操作锁
        self._csv_enabled = True  # CSV保存开关

        # 处理保存路径
        if save_path is None:
            save_path = os.getcwd()
        
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            print(f"创建保存目录: {save_path}")
        
        self.save_path = save_path
        self._base_csv_name = output_csv  # 保存原始文件名
        
        unique_filename = get_unique_filename(output_csv)
        self.output_csv = unique_filename
        self.full_path = os.path.join(self.save_path, self.output_csv)

        self.csv_file = open(self.full_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.shared_data = {
            "joint1": {
                "knee_angle": None, "v": None, "t": None,
                "p_des": None, "v_des": None, "kp": None,
                "kd": None, "t_ff": None, "mode": None
            },
            "joint2": {
                "knee_angle": None, "v": None, "t": None,
                "p_des": None, "v_des": None, "kp": None,
                "kd": None, "t_ff": None, "mode": None
            }
        }
        
        self._write_csv_headers()

    def _write_csv_headers(self):
        """写入CSV表头"""
        headers = ["timestamp"]
        for joint in ["joint1", "joint2"]:
            headers.extend([
                f"{joint}_knee_angle", f"{joint}_v", f"{joint}_t",
                f"{joint}_p_des", f"{joint}_v_des", f"{joint}_kp",
                f"{joint}_kd", f"{joint}_t_ff", f"{joint}_mode"
            ])
        self.csv_writer.writerow(headers)

    # ==================== CSV保存接口 ====================
    
    def set_csv_enabled(self, enabled: bool):
        """启用/禁用CSV保存"""
        with self._csv_lock:
            self._csv_enabled = enabled
        print(f"CSV保存: {'启用' if enabled else '禁用'}")

    def is_csv_enabled(self) -> bool:
        """获取CSV保存状态"""
        return self._csv_enabled

    def set_save_path(self, new_path: str, switch_file: bool = True):
        """
        修改保存路径
        
        参数:
            new_path: 新的保存路径
            switch_file: 是否立即切换到新路径下的新文件
        """
        if not os.path.exists(new_path):
            os.makedirs(new_path)
            print(f"创建保存目录: {new_path}")
        
        self.save_path = new_path
        
        if switch_file:
            self.switch_csv_file(self._base_csv_name)

    def switch_csv_file(self, new_filename: str = None, add_timestamp: bool = True):
        """
        切换到新的CSV文件
        
        参数:
            new_filename: 新文件名，None则使用原始文件名
            add_timestamp: 是否添加时间戳
        """
        with self._csv_lock:
            # 关闭旧文件
            if self.csv_file and not self.csv_file.closed:
                self.csv_file.flush()
                self.csv_file.close()
                print(f"已关闭: {self.full_path}")
            
            # 确定新文件名
            if new_filename is None:
                new_filename = self._base_csv_name
            else:
                self._base_csv_name = new_filename
            
            if add_timestamp:
                self.output_csv = get_unique_filename(new_filename)
            else:
                self.output_csv = new_filename
            
            self.full_path = os.path.join(self.save_path, self.output_csv)
            
            # 打开新文件
            self.csv_file = open(self.full_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self._write_csv_headers()
            
            print(f"已切换到新文件: {self.full_path}")

    def get_save_path(self) -> str:
        """获取当前保存路径"""
        return self.save_path
    
    def get_full_file_path(self) -> str:
        """获取完整的文件路径"""
        return self.full_path

    def get_csv_info(self) -> dict:
        """获取当前CSV配置信息"""
        return {
            "save_path": self.save_path,
            "filename": self.output_csv,
            "full_path": self.full_path,
            "enabled": self._csv_enabled
        }

    # ==================== 其他方法 ====================

    def get_shared_data(self):
        import copy
        return copy.deepcopy(self.shared_data)

    def set_data_callback(self, callback):
        self.data_callback = callback

    def send_line(self, data_str: str):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial not opened")
        if not data_str.endswith("\n"):
            data_str += "\n"
        self.ser.write(data_str.encode("utf-8"))

    def send_floats(self, data, fmt="{:.4f}"):
        str_list = [fmt.format(x) for x in data]
        line = ",".join(str_list)
        self.send_line(line)

    def start(self):
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print(f"串口 {self.ser.port} 开始接收")
        print(f"数据保存至: {self.full_path}")

    def stop(self):
        self.running = False
        time.sleep(0.2)
        if self.ser.is_open:
            self.ser.close()
        with self._csv_lock:
            if self.csv_file and not self.csv_file.closed:
                self.csv_file.close()
        print(f"串口停止，数据已保存至: {self.full_path}")

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
            
            row_data = [timestamp]
            
            if len(joints) > 0:
                j1 = joints[0]
                row_data.extend([
                    j1["knee_angle"], j1["v"], j1["t"],
                    j1["p_des"], j1["v_des"], j1["kp"], j1["kd"], j1["t_ff"], j1["mode"]
                ])
                self.shared_data["joint1"] = j1
            else:
                row_data.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0])
            
            if len(joints) > 1:
                j2 = joints[1]
                row_data.extend([
                    j2["knee_angle"], j2["v"], j2["t"],
                    j2["p_des"], j2["v_des"], j2["kp"], j2["kd"], j2["t_ff"], j2["mode"]
                ])
                self.shared_data["joint2"] = j2
            else:
                row_data.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0])
            
            # 线程安全的CSV写入
            with self._csv_lock:
                if self._csv_enabled and self.csv_file and not self.csv_file.closed:
                    self.csv_writer.writerow(row_data)
                    self.csv_file.flush()
                            
            if self.data_callback:
                self.data_callback(timestamp, joints)
                
        except Exception as e:
            print(f"串口错误: {e}")


if __name__ == "__main__":
    receiver = SerialReceiver(
        port="COM19", 
        baudrate=460800, 
        output_csv="joint.csv",
        save_path="./data"
    )
    receiver.start()

    try:
        while True:
            time.sleep(0.1)
            
            # 示例：外部控制CSV保存
            # receiver.set_csv_enabled(False)          # 暂停保存
            # receiver.set_save_path("./new_data")     # 修改路径
            # receiver.switch_csv_file("new_log.csv")  # 切换文件
            # print(receiver.get_csv_info())           # 获取配置
            
    except KeyboardInterrupt:
        receiver.stop()