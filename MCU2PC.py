import serial
import struct
import threading
import csv
import time
from datetime import datetime
import os

# ========== 1锔忊儯 Joint 瑙ｅ寘鍑芥暟 ==========
def unpack_joint(payload):

    if len(payload) != 32:
        raise ValueError(f"单个 joint 数据应为 32 字节，实际为 {len(payload)} 字节")

    ret0, ret1, ret2, p_des, v_des, kp, kd, t_ff = struct.unpack('<8f', payload)

    return {
        "ret": [ret0, ret1, ret2],
        "p_des": p_des,
        "v_des": v_des,
        "kp": kp,
        "kd": kd,
        "t_ff": t_ff
    }


def unpack_frame(payload):
    joint_size = 32
    joints = []
    for i in range(0, len(payload), joint_size):
        block = payload[i:i+joint_size]
        if len(block) == joint_size:
            joints.append(unpack_joint(block))
    return joints

def get_unique_filename(self, path: str) -> str:
    base, ext = os.path.splitext(path)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{base}_{timestamp}{ext}"



class SerialReceiver:
    def __init__(self, port, baudrate=115200, output_csv="data_log.csv"):
        self.ser = serial.Serial(port, baudrate, timeout=0.05)
        self.buffer = bytearray()
        self.running = True
        self.output_csv = get_unique_filename(output_csv)

        self.csv_file = open(self.output_csv, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp", "joint_id",
            "ret0", "ret1", "ret2", "p_des", "v_des", "kp", "kd", "t_ff"
        ])

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

            # 鏍￠獙甯у熬
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
            for idx, j in enumerate(joints):
                self.csv_writer.writerow([
                    timestamp, idx + 1,
                    j["ret"][0], j["ret"][1], j["ret"][2],
                    j["p_des"], j["v_des"], j["kp"], j["kd"], j["t_ff"]
                ])
            self.csv_file.flush()
        except Exception as e:
            print(f"串口错误: {e}")


if __name__ == "__main__":
    receiver = SerialReceiver(port="COM5", baudrate=115200, output_csv="joint.csv")
    receiver.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        receiver.stop()
