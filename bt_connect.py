import bluetooth
import struct
import time
import threading
from queue import Queue
import csv
from datetime import datetime
from typing import Callable, Optional, List, Dict, Any
from concurrent.futures import ThreadPoolExecutor, as_completed
import os
from dataclasses import dataclass


# 协议常量定义
START_FLAG = b'\x3A'  # 包头: 0x3A
END_FLAG = b'\x0D\x0A'  # 包尾: 0x0D0A
PACKET_SIZE = 47  # 数据包固定大小


class IMUSensorData:
    """IMU传感器数据类"""
    def __init__(self, device_id: str, timestamp: float,
                 gyro: tuple, acc: tuple, mag: tuple,
                 quat: tuple, lin_acc: tuple,
                 system_time: str):
        self.device_id = device_id
        self.timestamp = timestamp
        self.gyro_x, self.gyro_y, self.gyro_z = gyro
        self.acc_x, self.acc_y, self.acc_z = acc
        self.mag_x, self.mag_y, self.mag_z = mag
        self.quat_w, self.quat_x, self.quat_y, self.quat_z = quat
        self.lin_acc_x, self.lin_acc_y, self.lin_acc_z = lin_acc
        self.system_time = system_time
        
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典格式"""
        return {
            'device_id': self.device_id,
            'system_time': self.system_time,
            'timestamp': self.timestamp,
            'gyro': (self.gyro_x, self.gyro_y, self.gyro_z),
            'acc': (self.acc_x, self.acc_y, self.acc_z),
            'mag': (self.mag_x, self.mag_y, self.mag_z),
            'quat': (self.quat_w, self.quat_x, self.quat_y, self.quat_z),
            'lin_acc': (self.lin_acc_x, self.lin_acc_y, self.lin_acc_z)
        }
    
    def to_list(self) -> List:
        """转换为列表格式（用于CSV写入）"""
        return [
            self.system_time, self.timestamp,
            self.gyro_x, self.gyro_y, self.gyro_z,
            self.acc_x, self.acc_y, self.acc_z,
            self.mag_x, self.mag_y, self.mag_z,
            self.quat_w, self.quat_x, self.quat_y, self.quat_z,
            self.lin_acc_x, self.lin_acc_y, self.lin_acc_z
        ]


class IMUDevice:
    """单个IMU设备类"""
    def __init__(self, device_id: str, mac_address: str, 
                 data_callback: Optional[Callable[[IMUSensorData], None]] = None,
                 error_callback: Optional[Callable[[str, str], None]] = None,
                 csv_file_path: Optional[str] = "./imu.csv"):
        """
        初始化IMU设备
        
        Args:
            device_id: 设备唯一标识符
            mac_address: 蓝牙MAC地址
            data_callback: 数据回调函数，接收IMUSensorData对象
            error_callback: 错误回调函数，接收(device_id, error_message)
        """
        self.device_id = device_id
        self.mac_address = mac_address
        self.data_callback = data_callback
        self.error_callback = error_callback
        
        self.sock = None
        self.buffer = []
        self.stop_event = threading.Event()
        self.receive_thread = None
        self.is_connected = False
        self._connection_lock = threading.Lock()

        self.csv_enabled = True  # 是否保存CSV
        self.csv_file_path_base = csv_file_path  # 原始文件路径模板
        self.csv_file_path = None
        self.csv_file = None
        self.csv_writer = None

    def _open_new_csv(self):
        if not self.csv_enabled:
            return
        self.csv_file_path = self._get_unique_filename(self.csv_file_path_base)
        self.csv_file = open(self.csv_file_path, mode='w',newline='',encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "System_Time", "Timestamp",
            "Gyro_X", "Gyro_Y", "Gyro_Z",
            "Acc_X", "Acc_Y", "Acc_Z",
            "Mag_X", "Mag_Y", "Mag_Z",
            "Quat_W", "Quat_X", "Quat_Y", "Quat_Z",
            "Linear_Acc_X", "Linear_Acc_Y", "Linear_Acc_Z"
        ])
        print(f"🟢 已创建新的 CSV 文件: {os.path.basename(self.csv_file_path)}")

    def _close_csv(self):
        if self.csv_file:
            self.csv_file.close()
            print(f"🟡 已关闭 CSV 文件: {os.path.basename(self.csv_file_path)}")
            self.csv_file = None
            self.csv_writer = None

    def _get_unique_filename(self, path: str) -> str:
        """生成唯一文件名"""
        base, ext = os.path.splitext(path)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        mac_suffix = self.mac_address.replace(':', '')[-6:]
        return f"{base}_{mac_suffix}_{timestamp}{ext}"
    
    def enable_csv(self, enable: bool):
        """打开或关闭CSV保存"""
        self.csv_enabled = enable
        print(f"CSV 保存 {'已开启' if enable else '已关闭'}")

    def save_to_csv(self, data):
        """保存数据到CSV文件"""
        if not self.csv_enabled or not self.csv_writer:
            return
        try:
            self.csv_writer.writerow(data.to_list())
        except Exception as e:
            if self.error_callback:
                self.error_callback(self.device_id, f"CSV写入错误: {e}")

    def start_receiving(self):
        """启动接收线程"""
        if not self.is_connected:
            print(f"✗ 设备 {self.device_id} 未连接，无法启动接收")
            return
        if self.receive_thread and self.receive_thread.is_alive():
            print(f"⚠ 设备 {self.device_id} 接收线程已在运行")
            return

        # ✅ 每次启动都重新创建一个新CSV
        self._open_new_csv()

        self.stop_event.clear()
        self.receive_thread = threading.Thread(
            target=self._receive_data,
            name=f"IMU-{self.device_id}",
            daemon=True
        )
        self.receive_thread.start()
        print(f"✓ 设备 {self.device_id} 开始接收数据")


    def connect(self, port: int = 1, timeout: float = 10.0) -> bool:
        """
        连接到IMU设备
        
        Args:
            port: RFCOMM端口号
            timeout: 连接超时时间（秒）
        """
        with self._connection_lock:
            if self.is_connected:
                print(f"⚠ 设备 {self.device_id} 已经连接")
                return True
            
            try:
                print(f"⏳ 正在连接设备 {self.device_id} ({self.mac_address})...")
                self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                
                # 设置超时
                self.sock.settimeout(timeout)
                self.sock.connect((self.mac_address, port))
                
                # 连接成功后，设置为非阻塞模式以便接收数据
                self.sock.settimeout(1.0)
                
                self.is_connected = True
                print(f"✓ 设备 {self.device_id} ({self.mac_address}) 连接成功")
                return True
                
            except bluetooth.BluetoothError as e:
                error_msg = f"蓝牙连接失败: {e}"
                print(f"✗ 设备 {self.device_id} {error_msg}")
                if self.error_callback:
                    self.error_callback(self.device_id, error_msg)
                return False
            except Exception as e:
                error_msg = f"连接失败: {e}"
                print(f"✗ 设备 {self.device_id} {error_msg}")
                if self.error_callback:
                    self.error_callback(self.device_id, error_msg)
                return False
    
    
    def stop_receiving(self):
        """停止数据接收"""
        self.stop_event.set()
        if self.receive_thread:
            self.receive_thread.join(timeout=2)
        print(f"✓ 设备 {self.device_id} 停止接收数据")
    
    def disconnect(self):
        """断开连接"""
        self.stop_receiving()
        with self._connection_lock:
            if self.sock:
                try:
                    self.sock.close()
                    self.is_connected = False
                    print(f"✓ 设备 {self.device_id} 断开连接")
                except Exception as e:
                    print(f"✗ 设备 {self.device_id} 断开连接时出错: {e}")
            self.sock = None
    
    def _unpack_packet(self, packet: bytes) -> Optional[IMUSensorData]:
        """解包数据"""
        try:
            data = packet[1:-4]
            fmt = '<HHHIhhhhhhhhhhhhhhhh'
            expected_size = struct.calcsize(fmt)
            
            if len(data) != expected_size:
                return None
            
            values = struct.unpack(fmt, data)
            timestamp = values[3] / 400.0
            gyro = tuple(v * 1e-3 for v in values[4:7])
            acc = tuple(v * 1e-3 for v in values[7:10])
            mag = tuple(v * 1e-2 for v in values[10:13])
            quat = tuple(v * 1e-4 for v in values[13:17])
            lin_acc = tuple(v * 1e-3 for v in values[17:20])
            system_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            self.save_to_csv(IMUSensorData(
                device_id=self.device_id,
                timestamp=timestamp,
                gyro=gyro,
                acc=acc,
                mag=mag,
                quat=quat,
                lin_acc=lin_acc,
                system_time=system_time
            ))

            return IMUSensorData(
                device_id=self.device_id,
                timestamp=timestamp,
                gyro=gyro,
                acc=acc,
                mag=mag,
                quat=quat,
                lin_acc=lin_acc,
                system_time=system_time
            )
        except Exception as e:
            error_msg = f"解包错误: {e}"
            if self.error_callback:
                self.error_callback(self.device_id, error_msg)
            return None


    def _receive_data(self):
        """数据接收线程函数"""
        while not self.stop_event.is_set():
            try:
                data = self.sock.recv(1024)
                if data:
                    self.buffer.append(data)
                    
                    while len(self.buffer) > 0:
                        combined_data = b''.join(self.buffer)
                        
                        if len(combined_data) >= PACKET_SIZE:
                            # 验证包头和包尾
                            if combined_data[:1] != START_FLAG:
                                self.buffer.clear()
                                break
                            
                            if combined_data[PACKET_SIZE-2:PACKET_SIZE] != END_FLAG:
                                self.buffer.clear()
                                break
                            
                            # 提取并解包数据
                            packet = combined_data[:PACKET_SIZE]
                            sensor_data = self._unpack_packet(packet)
                            
                            if sensor_data and self.data_callback:
                                self.data_callback(sensor_data)
                            
                            # 处理剩余数据
                            remaining_data = combined_data[PACKET_SIZE:]
                            self.buffer.clear()
                            if len(remaining_data) > 0:
                                self.buffer.append(remaining_data)
                        else:
                            break
                            
            except bluetooth.BluetoothError as e:
                if not self.stop_event.is_set():
                    error_msg = f"蓝牙错误: {e}"
                    if self.error_callback:
                        self.error_callback(self.device_id, error_msg)
                break
            except Exception as e:
                if not self.stop_event.is_set():
                    error_msg = f"接收线程错误: {e}"
                    if self.error_callback:
                        self.error_callback(self.device_id, error_msg)
                break


class MultiIMUManager:
    """多IMU设备管理器"""
    def __init__(self):
        """
        初始化多IMU管理器
        """
        self.devices: Dict[str, IMUDevice] = {}

        self.data_queue = Queue()
        self.external_callbacks: List[Callable[[IMUSensorData], None]] = []
        self.latest_data: Dict[str, IMUSensorData] = {}

    def get_latest_data(self, device_id: str) -> Dict[str, Optional[IMUSensorData]]:
        """
        获取指定设备的最新数据
        
        Args:
            device_id: 设备ID
        
        Returns:
            IMUSensorData对象或None
        """
        connected_ids = self.get_connected_devices()
        return {
            device_id: data 
            for device_id, data in self.latest_data.items()
            if device_id in connected_ids
        }
    
    def add_device(self, device_id: str, mac_address: str) -> bool:
        """添加IMU设备"""
        if device_id in self.devices:
            print(f"✗ 设备 {device_id} 已存在")
            return False
        
        device = IMUDevice(
            device_id=device_id,
            mac_address=mac_address,
            data_callback=self._on_data,
            error_callback=self._on_error
        )
        self.devices[device_id] = device
        print(f"✓ 添加设备 {device_id} ({mac_address})")
        return True
    
    def _on_data(self, data: IMUSensorData):
        """内部数据回调处理"""
        # 放入队列
        self.data_queue.put(data)
        
        # 调用外部注册的回调函数
        for callback in self.external_callbacks:
            callback(data)

    def register_callback(self, callback: Callable[[IMUSensorData], None]):
        """
        注册外部数据回调函数
        
        Args:
            callback: 回调函数，接收IMUSensorData对象
        """
        self.external_callbacks.append(callback)
    
    def connect_all(self, timeout: float = 10.0, parallel: bool = False) -> Dict[str, bool]:
        """
        连接所有设备
        
        Args:
            timeout: 每个设备的连接超时时间（秒）
            parallel: 是否并行连接（True=并行，False=串行）
        
        Returns:
            Dict[device_id, success]: 每个设备的连接状态
        """
        results = {}
        
        if not self.devices:
            print("⚠ 没有要连接的设备")
            return results
        
        if parallel:
            # 并行连接所有设备
            print(f"\n🔗 开始并行连接 {len(self.devices)} 个设备...")
            with ThreadPoolExecutor(max_workers=len(self.devices)) as executor:
                future_to_device = {
                    executor.submit(device.connect, 1, timeout): device_id
                    for device_id, device in self.devices.items()
                }
                
                for future in as_completed(future_to_device):
                    device_id = future_to_device[future]
                    try:
                        results[device_id] = future.result()
                    except Exception as e:
                        print(f"✗ 设备 {device_id} 连接异常: {e}")
                        results[device_id] = False
        else:
            # 串行连接设备
            print(f"\n🔗 开始串行连接 {len(self.devices)} 个设备...")
            port = 1
            for device_id, device in self.devices.items():
                results[device_id] = device.connect(port=port, timeout=timeout)
                port += 1
                time.sleep(0.5)  # 串行连接时稍微延迟
        
        # 统计连接结果
        success_count = sum(1 for v in results.values() if v)
        print(f"\n📊 连接完成: {success_count}/{len(results)} 个设备成功连接")
        
        return results
    
    def connect_device(self, device_id: str, timeout: float = 10.0) -> bool:
        """
        连接单个设备
        
        Args:
            device_id: 设备ID
            timeout: 连接超时时间（秒）
        """
        if device_id not in self.devices:
            print(f"✗ 设备 {device_id} 不存在")
            return False
        
        return self.devices[device_id].connect(timeout=timeout)
    
    def start_all(self):
        """启动所有已连接设备的数据接收"""
        started_count = 0
        for device in self.devices.values():
            if device.is_connected:
                device.start_receiving()
                started_count += 1
        
        print(f"\n▶ 已启动 {started_count}/{len(self.devices)} 个设备的数据接收")
    
    def start_device(self, device_id: str):
        """启动单个设备的数据接收"""
        if device_id in self.devices:
            self.devices[device_id].start_receiving()
    
    def stop_all(self):
        """停止所有设备的数据接收"""
        for device in self.devices.values():
            device.stop_receiving()
        print(f"\n⏸ 已停止所有设备的数据接收")
    
    def disconnect_all(self):
        """断开所有设备连接"""
        for device in self.devices.values():
            device.disconnect()
        print(f"\n🔌 已断开所有设备连接")
    
    def get_connected_devices(self) -> List[str]:
        """获取已连接的设备ID列表"""
        return [device_id for device_id, device in self.devices.items() if device.is_connected]
    
    def get_device_status(self) -> Dict[str, Dict[str, Any]]:
        """获取所有设备的状态"""
        status = {}
        for device_id, device in self.devices.items():
            status[device_id] = {
                'mac_address': device.mac_address,
                'is_connected': device.is_connected,
                'is_receiving': device.receive_thread.is_alive() if device.receive_thread else False
            }
        return status
    
    def get_data(self, timeout: Optional[float] = None) -> Optional[IMUSensorData]:
        """
        从队列获取数据（阻塞式）
        
        Args:
            timeout: 超时时间（秒），None表示一直等待
        
        Returns:
            IMUSensorData对象或None
        """
        try:
            return self.data_queue.get(timeout=timeout)
        except:
            return None
    
    def _on_error(self, device_id: str, error_msg: str):
        """错误处理回调"""
        print(f"✗ [{device_id}] 错误: {error_msg}")


# 使用示例
if __name__ == "__main__":
    # 创建管理器
    manager = MultiIMUManager()
    
    # 添加多个IMU设备
    manager.add_device("IMU_1", "00:04:3E:6C:51:C1")
    manager.add_device("IMU_2", "00:04:3E:86:27:F0")  # 替换为实际MAC地址
    manager.add_device("IMU_3", "00:04:3E:86:27:ED")  # 可以添加更多设备
    
    # 注册自定义回调函数（可选）
    # def my_callback(data: IMUSensorData):
    #     print(f"[{data.device_id}] Acc: ({data.acc_x:.3f}, {data.acc_y:.3f}, {data.acc_z:.3f})")
    
    # manager.register_callback(my_callback)
    
    # 并行连接所有设备（推荐，更快）
    print("\n" + "="*50)
    connection_results = manager.connect_all(timeout=10.0, parallel=True)
    print("="*50)
    
    # 打印连接状态
    print("\n设备状态:")
    for device_id, status in manager.get_device_status().items():
        status_icon = "✓" if status['is_connected'] else "✗"
        print(f"  {status_icon} {device_id}: {status['mac_address']} - {'已连接' if status['is_connected'] else '未连接'}")
    
    # 只启动成功连接的设备
    connected_devices = manager.get_connected_devices()
    if connected_devices:
        print(f"\n已连接的设备: {', '.join(connected_devices)}")
        manager.start_all()
        
        # 主循环
        try:
            print("\n正在接收数据，按Ctrl+C停止...\n")
            while True:
                # 方式1: 从队列获取数据
                data = manager.get_data(timeout=1.0)
                # if data:
                #     print(f"[队列] {data.device_id}: 时间戳={data.timestamp:.3f}")
                
                # 方式2: 数据会自动通过回调函数处理
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n\n正在退出...")
        finally:
            manager.stop_all()
            manager.disconnect_all()
            print("程序已退出")
    else:
        print("\n⚠ 没有设备成功连接，程序退出")

