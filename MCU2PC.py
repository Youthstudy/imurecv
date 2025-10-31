"""
PC端可靠串口通信程序
功能：与MCU进行双向通信，支持数据校验、确认应答、超时重传
需要安装：pip install pyserial
"""

import serial
import time
import threading
import struct
from queue import Queue
from enum import Enum

# 通信协议定义
FRAME_HEADER = 0xAA
FRAME_TAIL = 0x55
MAX_DATA_LEN = 128
TIMEOUT_SEC = 1.0
MAX_RETRY = 3

class CmdType(Enum):
    CMD_DATA = 0x01
    CMD_ACK = 0x02
    CMD_NACK = 0x03
    CMD_HEARTBEAT = 0x04

class DataFrame:
    """数据帧类"""
    def __init__(self, seq=0, cmd=CmdType.CMD_DATA, data=b''):
        self.header = FRAME_HEADER
        self.seq = seq
        self.cmd = cmd.value if isinstance(cmd, CmdType) else cmd
        self.len = len(data)
        self.data = data
        self.checksum = 0
        self.tail = FRAME_TAIL
        self._calculate_checksum()
    
    def _calculate_checksum(self):
        """计算校验和"""
        checksum = self.seq ^ self.cmd ^ self.len
        for byte in self.data:
            checksum ^= byte
        self.checksum = checksum
    
    def to_bytes(self):
        """将数据帧转换为字节串"""
        frame = struct.pack('BBBB', self.header, self.seq, self.cmd, self.len)
        frame += self.data
        frame += struct.pack('BB', self.checksum, self.tail)
        return frame
    
    @staticmethod
    def from_bytes(data):
        """从字节串解析数据帧"""
        if len(data) < 6:
            return None
        
        header = data[0]
        seq = data[1]
        cmd = data[2]
        length = data[3]
        
        if header != FRAME_HEADER:
            return None
        
        if len(data) < 6 + length:
            return None
        
        payload = data[4:4+length]
        checksum = data[4+length]
        tail = data[5+length]
        
        if tail != FRAME_TAIL:
            return None
        
        # 验证校验和
        calc_checksum = seq ^ cmd ^ length
        for byte in payload:
            calc_checksum ^= byte
        
        if calc_checksum != checksum:
            return None
        
        frame = DataFrame(seq, cmd, payload)
        return frame

class SerialComm:
    """串口通信类"""
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.tx_seq = 0
        self.rx_seq = 0
        self.running = False
        self.rx_thread = None
        self.tx_queue = Queue()
        self.ack_events = {}  # 存储每个序列号的事件
        self.received_data_callback = None
        self.rx_buffer = bytearray()
        
    def open(self):
        """打开串口"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"Serial port {self.port} opened successfully at {self.baudrate} baud")
            self.running = True
            self.rx_thread = threading.Thread(target=self._receive_thread, daemon=True)
            self.rx_thread.start()
            return True
        except Exception as e:
            print(f"Failed to open serial port: {e}")
            return False
    
    def close(self):
        """关闭串口"""
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=2)
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Serial port closed")
    
    def _receive_thread(self):
        """接收线程"""
        while self.running:
            try:
                if self.serial and self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    self.rx_buffer.extend(data)
                    self._parse_buffer()
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"Receive error: {e}")
                time.sleep(0.1)
    
    def _parse_buffer(self):
        """解析接收缓冲区"""
        while len(self.rx_buffer) >= 6:
            # 查找帧头
            header_index = self.rx_buffer.find(FRAME_HEADER)
            if header_index == -1:
                self.rx_buffer.clear()
                break
            
            # 删除帧头之前的数据
            if header_index > 0:
                self.rx_buffer = self.rx_buffer[header_index:]
            
            # 检查是否有完整帧
            if len(self.rx_buffer) < 6:
                break
            
            data_len = self.rx_buffer[3]
            frame_len = 6 + data_len
            
            if len(self.rx_buffer) < frame_len:
                break
            
            # 提取完整帧
            frame_data = bytes(self.rx_buffer[:frame_len])
            self.rx_buffer = self.rx_buffer[frame_len:]
            
            # 解析数据帧
            frame = DataFrame.from_bytes(frame_data)
            if frame:
                self._process_frame(frame)
            else:
                print("Invalid frame received")
    
    def _process_frame(self, frame):
        """处理接收到的数据帧"""
        if frame.cmd == CmdType.CMD_DATA.value:
            print(f"Received DATA (seq={frame.seq}, len={frame.len}): {frame.data.hex()}")
            # 发送ACK
            self.send_ack(frame.seq)
            # 回调处理
            if self.received_data_callback:
                self.received_data_callback(frame.data)
        
        elif frame.cmd == CmdType.CMD_ACK.value:
            ack_seq = frame.data[0] if len(frame.data) > 0 else 0
            print(f"Received ACK for seq={ack_seq}")
            # 触发对应序列号的事件
            if ack_seq in self.ack_events:
                self.ack_events[ack_seq].set()
        
        elif frame.cmd == CmdType.CMD_NACK.value:
            nack_seq = frame.data[0] if len(frame.data) > 0 else 0
            print(f"Received NACK for seq={nack_seq}")
            # 可以触发重传逻辑
        
        elif frame.cmd == CmdType.CMD_HEARTBEAT.value:
            print("Received HEARTBEAT")
            # 回应心跳
            self.send_heartbeat()
    
    def send_frame(self, cmd, data=b''):
        """发送数据帧"""
        frame = DataFrame(self.tx_seq, cmd, data)
        self.tx_seq = (self.tx_seq + 1) % 256
        
        try:
            self.serial.write(frame.to_bytes())
            return frame.seq
        except Exception as e:
            print(f"Send error: {e}")
            return -1
    
    def send_ack(self, seq):
        """发送ACK确认"""
        self.send_frame(CmdType.CMD_ACK, bytes([seq]))
    
    def send_nack(self, seq):
        """发送NACK否定"""
        self.send_frame(CmdType.CMD_NACK, bytes([seq]))
    
    def send_heartbeat(self):
        """发送心跳包"""
        self.send_frame(CmdType.CMD_HEARTBEAT)
    
    def reliable_send(self, data):
        """可靠发送数据（带重传）"""
        if len(data) > MAX_DATA_LEN:
            print(f"Data too long: {len(data)} > {MAX_DATA_LEN}")
            return False
        
        retry = 0
        while retry < MAX_RETRY:
            # 发送数据
            seq = self.send_frame(CmdType.CMD_DATA, data)
            if seq < 0:
                retry += 1
                continue
            
            # 创建ACK等待事件
            event = threading.Event()
            self.ack_events[seq] = event
            
            # 等待ACK
            if event.wait(timeout=TIMEOUT_SEC):
                print(f"Data sent successfully (seq={seq})")
                del self.ack_events[seq]
                return True
            else:
                print(f"Timeout waiting for ACK (seq={seq}), retry {retry+1}/{MAX_RETRY}")
                del self.ack_events[seq]
                retry += 1
        
        print("Failed to send data after maximum retries")
        return False
    
    def set_data_callback(self, callback):
        """设置数据接收回调函数"""
        self.received_data_callback = callback

def main():
    """主函数示例"""
    # 创建串口通信对象
    comm = SerialComm(port='COM3', baudrate=115200)  # Windows
    
    # 打开串口
    if not comm.open():
        return
    
    # 设置接收回调
    def on_data_received(data):
        print(f"Application received: {data.hex()}")
    
    comm.set_data_callback(on_data_received)
    
    try:
        # 示例1：发送文本数据
        test_text = "Hello MCU!"
        print(f"\nSending text: {test_text}")
        comm.reliable_send(test_text.encode())
        time.sleep(1)
        
        # 示例2：发送二进制数据
        test_binary = bytes([0x01, 0x02, 0x03, 0x04, 0x05])
        print(f"\nSending binary: {test_binary.hex()}")
        comm.reliable_send(test_binary)
        time.sleep(1)
        
        # 示例3：发送心跳包
        print("\nSending heartbeat")
        comm.send_heartbeat()
        time.sleep(1)
        
        # 保持运行接收数据
        print("\nListening for incoming data... (Press Ctrl+C to exit)")
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        comm.close()

if __name__ == "__main__":
    main()

