import asyncio
from bleak import BleakScanner, BleakClient
from datetime import datetime 
import struct
from collections import deque
import csv
import os
import time


class BluetoothIMU:
    def __init__(self, mac_address, path='imu.csv', save_data=False, data_len=41, buffer_size=1000, batch_size=10):
        self.mac_address = mac_address
        self.sock = None
        self.running = False
        self.START_FLAG = b'\x3A'  # 包头: 0x3A
        self.END_FLAG = b'\x0D\x0A'  # 包尾: 0x0D0A (即 '\r')
        self.csv_file_path = self.getUniqueFilename(path)
        self.data_len = data_len
        
        # ✅ 优化：使用 deque 替代 asyncio.Queue（更快）
        self.buffer = deque(maxlen=buffer_size)
        self.buffer_lock = asyncio.Lock()
        
        self.saveflag = save_data
        self.step = 0
        self.raw_buffer = bytearray()
        self.unpacked_data = []
        
        # ✅ 添加客户端对象和连接状态
        self.client = None
        self.is_connected = False
        
        # ✅ 批量写入优化
        self.batch_size = batch_size
        self.write_buffer = []
        self.csv_file = None
        self.csv_writer = None
        
        # ✅ 统计信息
        self.packet_count = 0
        self.error_count = 0
        self.last_timestamp = None
        self.dropped_packets = 0
        self.last_report_time = time.time()
        self.packets_per_second = 0
        self.total_received = 0

    @staticmethod
    async def scan_devices(timeout=5):
        """扫描 BLE 设备"""
        print(f"🔍 正在扫描 BLE 设备 ({timeout}s)...")
        devices = await BleakScanner.discover(timeout=timeout)
        if not devices:
            print("❌ 没有发现设备")
            return []
        
        print(f"\n找到 {len(devices)} 个设备:")
        for i, d in enumerate(devices):
            print(f"{i}: {d.name or '未知设备'} [{d.address}] RSSI: {d.rssi}")
        return devices

    def process_data(self, sender, data):
        """✅ 优化：直接添加到 deque"""
        try:
            self.buffer.append(data)
            self.total_received += 1
        except Exception as e:
            print(f"⚠️ [{self.mac_address}] 数据入队失败:", e)
            self.error_count += 1

    # ✅ 数据处理任务：持续从队列取数据、解析、打印/保存
    async def data_handler(self):
        print(f"🧩 [{self.mac_address}] 数据处理线程已启动")
        
        # 如果需要保存，打开CSV文件
        if self.saveflag:
            self._open_csv_file()
        
        while self.is_connected:
            try:
                # ✅ 批量处理数据
                batch = []
                while len(self.buffer) > 0 and len(batch) < 20:
                    try:
                        batch.append(self.buffer.popleft())
                    except IndexError:
                        break
                
                if batch:
                    for data in batch:
                        await self.handle_packet(data)
                else:
                    # 没有数据时短暂休眠
                    await asyncio.sleep(0.001)
                
                # ✅ 定期刷新CSV缓冲区
                if self.saveflag and len(self.write_buffer) >= self.batch_size:
                    self._flush_csv_buffer()
                
                # ✅ 定期报告统计
                await self._report_statistics()
                
            except Exception as e:
                print(f"⚠️ [{self.mac_address}] 数据处理错误: {e}")
                self.error_count += 1
        
        # 清理资源
        if self.saveflag:
            self._close_csv_file()
        print(f"🛑 [{self.mac_address}] 数据处理线程已停止")

    async def handle_packet(self, data):
        self.raw_buffer.extend(data)
        
        while len(self.raw_buffer) >= self.data_len:
            # 查找帧头
            start_index = self.raw_buffer.find(self.START_FLAG)

            if start_index == -1:
                if len(self.raw_buffer) > 0:
                    # print(f"❌ [{self.mac_address}] 未找到帧头 0x3A，丢弃数据")
                    self.error_count += 1
                self.raw_buffer.clear()
                break
            
            # 如果帧头不在开始位置，丢弃之前的数据
            if start_index > 0:
                # print(f"⚠️ [{self.mac_address}] 丢弃帧头前的数据")
                self.raw_buffer = self.raw_buffer[start_index:]
                self.error_count += 1

            # 查找帧尾（从帧头之后开始查找）
            end_index = self.raw_buffer.find(self.END_FLAG, 1)

            if end_index == -1:
                # 如果缓冲区过大，可能是数据损坏
                if len(self.raw_buffer) > 1024:
                    # print(f"❌ [{self.mac_address}] 缓冲区溢出，清空数据")
                    self.raw_buffer.clear()
                    self.error_count += 1
                break
            
            # 提取完整的数据包（包括帧头和帧尾）
            packet_end = end_index + len(self.END_FLAG)
            packet = bytes(self.raw_buffer[:packet_end])
            
            # 从缓冲区移除已处理的数据包
            self.raw_buffer = self.raw_buffer[packet_end:]

            # 验证数据包
            if packet[0:1] == self.START_FLAG and packet[-2:] == self.END_FLAG:
                # 解析数据包
                self.unpacked_data = self.unpack_data(packet)
                if self.unpacked_data:
                    self.packet_count += 1
                    
                    # ✅ 检测丢包
                    self._check_packet_loss(self.unpacked_data["timestamp"])
                    
                    if self.saveflag:
                        # 添加到批量写入缓冲区
                        self.write_buffer.append(self.unpacked_data)
            else:
                # print(f"❌ [{self.mac_address}] 数据包格式错误")
                self.error_count += 1

    def _check_packet_loss(self, current_timestamp):
        """✅ 检测丢包（基于时间戳连续性）"""
        if self.last_timestamp is not None:
            # 假设采样率为200Hz（每个包间隔0.005秒）
            expected_interval = 1/200  # 200Hz
            actual_interval = current_timestamp - self.last_timestamp
            
            # 如果间隔超过预期的1.5倍，认为可能丢包
            if actual_interval > expected_interval * 1.5:
                missed = int(actual_interval / expected_interval) - 1
                if missed > 0:
                    self.dropped_packets += missed
                    # print(f"⚠️ [{self.mac_address}] 检测到丢包 {missed} 个")
        
        self.last_timestamp = current_timestamp

    def _open_csv_file(self):
        """打开CSV文件"""
        try:
            file_exists = os.path.isfile(self.csv_file_path)
            self.csv_file = open(self.csv_file_path, mode='a', newline='', encoding='utf-8', buffering=8192)
            self.csv_writer = csv.writer(self.csv_file)
            
            if not file_exists:
                self.csv_writer.writerow([
                    "Timestamp", "imuTimestamp", "Gyro_X (rad/s)", "Gyro_Y (rad/s)", "Gyro_Z (rad/s)",
                    "Acc_X (g)", "Acc_Y (g)", "Acc_Z (g)",
                    "Mag_X (μT)", "Mag_Y (μT)", "Mag_Z (μT)",
                    "Euler_X", "Euler_Y", "Euler_Z",
                    "Quaternion_W", "Quaternion_X", "Quaternion_Y", "Quaternion_Z",
                    "Linear_Acc_X (g)", "Linear_Acc_Y (g)", "Linear_Acc_Z (g)"
                ])
        except Exception as e:
            print(f"❌ [{self.mac_address}] 打开CSV文件失败: {e}")

    def _flush_csv_buffer(self):
        """✅ 批量写入CSV"""
        if not self.write_buffer or not self.csv_writer:
            return
        
        try:
            for data in self.write_buffer:
                t = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                self.csv_writer.writerow([
                    t,
                    data["timestamp"],
                    *data["gyro"],
                    *data["acc"],
                    *data["mag"],
                    *data["euler"],
                    *data["quaternion"],
                    *data["linear_acceleration"],
                ])
            
            self.csv_file.flush()
            self.write_buffer.clear()
        except Exception as e:
            print(f"❌ [{self.mac_address}] 批量写入CSV失败: {e}")

    def _close_csv_file(self):
        """关闭CSV文件"""
        if self.write_buffer:
            self._flush_csv_buffer()
        
        if self.csv_file:
            self.csv_file.close()
            print(f"📝 [{self.mac_address}] CSV文件已保存: {self.csv_file_path}")

    async def _report_statistics(self):
        """✅ 定期报告统计信息"""
        current_time = time.time()
        if current_time - self.last_report_time >= 5.0:
            elapsed = current_time - self.last_report_time
            self.packets_per_second = self.packet_count / elapsed
            
            # 计算丢包率
            total_expected = self.packet_count + self.dropped_packets
            loss_rate = (self.dropped_packets / total_expected * 100) if total_expected > 0 else 0
            
            print(f"📊 [{self.mac_address}] 统计报告:")
            print(f"   ├─ 接收速率: {self.packets_per_second:.1f} pps")
            print(f"   ├─ 成功解析: {self.packet_count} 包")
            print(f"   ├─ 丢包数量: {self.dropped_packets} 包")
            print(f"   ├─ 丢包率: {loss_rate:.2f}%")
            print(f"   ├─ 解析错误: {self.error_count} 次")
            print(f"   ├─ 缓冲区使用: {len(self.buffer)}/{self.buffer.maxlen if hasattr(self.buffer, 'maxlen') else '∞'}")
            print(f"   └─ 总接收: {self.total_received} 次\n")
            
            # 重置计数器
            self.packet_count = 0
            self.error_count = 0
            self.dropped_packets = 0
            self.last_report_time = current_time

    def unpack_data(self, data):
        try:
            # 提取Sensor ID
            sensor_id = struct.unpack('<H', data[1:3])[0]
            # 提取指令号
            command_id = struct.unpack('<H', data[3:5])[0]
            # 提取数据长度
            data_length = struct.unpack('<H', data[5:7])[0]
            # 提取时间戳
            timestamp_sec = struct.unpack('<I', data[7:11])[0] / 400
            # 提取传感器数据
            sensor_data = data[11:-4]

            # ✅ 批量解包
            gyro = struct.unpack('<3h', sensor_data[0:6])
            acc = struct.unpack('<3h', sensor_data[6:12])
            mag = struct.unpack('<3h', sensor_data[12:18])
            quat = struct.unpack('<4h', sensor_data[18:26])
            euler = struct.unpack('<3h', sensor_data[26:32])
            lin_acc = struct.unpack('<3h', sensor_data[32:38])

            return {
                "sensor_id": sensor_id,
                "command_id": command_id,
                "data_length": data_length,
                "gyro": tuple(x * 1e-3 for x in gyro),
                "acc": tuple(x * 1e-3 for x in acc),
                "mag": tuple(x * 1e-2 for x in mag),
                "euler": tuple(x * 1e-4 for x in euler),
                "quaternion": tuple(x * 1e-4 for x in quat),
                "linear_acceleration": tuple(x * 1e-3 for x in lin_acc),
                "timestamp": timestamp_sec
            }
        except Exception as e:
            # print(f"❌ [{self.mac_address}] 解包失败: {e}")
            return None

    # ✅ 蓝牙连接与订阅
    async def connect_and_read(self):
        print(f"🔗 [{self.mac_address}] 尝试连接...")
        
        try:
            self.client = BleakClient(self.mac_address, timeout=20.0)
            await self.client.connect()
            
            if not self.client.is_connected:
                print(f"❌ [{self.mac_address}] 连接失败")
                return False
            
            self.is_connected = True
            print(f"✅ [{self.mac_address}] 连接成功！")

            # 寻找可通知特征
            readable_chars = [
                c for service in self.client.services
                for c in service.characteristics
                if "notify" in c.properties
            ]
            
            if not readable_chars:
                print(f"⚠️ [{self.mac_address}] 未发现可通知特征")
                return False

            char = readable_chars[0]
            print(f"🔔 [{self.mac_address}] 订阅 {char.uuid} 的通知...")

            await self.client.start_notify(char.uuid, self.process_data)

            # ✅ 运行数据处理任务
            await self.data_handler()
            
            return True
            
        except Exception as e:
            print(f"❌ [{self.mac_address}] 连接错误: {e}")
            self.is_connected = False
            return False

    async def disconnect(self):
        """✅ 断开连接"""
        if self.client and self.is_connected:
            self.is_connected = False
            try:
                await self.client.disconnect()
                print(f"🔌 [{self.mac_address}] 已断开连接")
            except Exception as e:
                print(f"⚠️ [{self.mac_address}] 断开连接时出错: {e}")
                    
    def getUniqueFilename(self, path):
        base, ext = os.path.splitext(path)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        mac_suffix = self.mac_address.replace(':', '')[-6:]  # 取MAC地址后6位
        return f"{base}_{mac_suffix}_{timestamp}{ext}"


# ===== 多设备管理器 =====
class MultiIMUManager:
    def __init__(self):
        self.imu_devices = []
        self.tasks = []
    
    def add_device(self, mac_address, save_data=True, path=None, buffer_size=1000, batch_size=10):
        """添加IMU设备"""
        if path is None:
            path = f"imu.csv"
        
        imu = BluetoothIMU(
            mac_address=mac_address,
            save_data=save_data,
            path=path,
            buffer_size=buffer_size,
            batch_size=batch_size
        )
        self.imu_devices.append(imu)
        print(f"➕ 添加设备: {mac_address}")
        return imu
    
    async def connect_all(self):
        """并发连接所有设备"""
        print(f"\n🚀 开始并发连接 {len(self.imu_devices)} 个设备...\n")
        
        # 创建所有连接任务
        self.tasks = [
            asyncio.create_task(imu.connect_and_read())
            for imu in self.imu_devices
        ]
        
        # 并发执行
        try:
            await asyncio.gather(*self.tasks)
        except KeyboardInterrupt:
            print("⚠️ 收到中断信号，正在断开所有连接...")
            await self.disconnect_all()
    
    async def disconnect_all(self):
        """断开所有设备"""
        print("\n🔌 断开所有设备...")
        disconnect_tasks = [imu.disconnect() for imu in self.imu_devices]
        await asyncio.gather(*disconnect_tasks, return_exceptions=True)
        print("✅ 所有设备已断开")
    
    def print_status(self):
        """打印所有设备状态"""
        print("" + "="*60)
        print("设备连接状态:")
        print("="*60)
        for imu in self.imu_devices:
            status = "✅ 已连接" if imu.is_connected else "❌ 未连接"
            print(f"{status} - {imu.mac_address}")
        print("="*60 + "\n")



# ===== 使用示例 =====
async def main():
    # 方式1: 手动指定多个设备地址
    imu_addresses = [
        "00:04:3E:6C:51:C1",
        "00:04:3E:86:27:F0",
        "00:04:3E:86:27:ED",
    ]
    
    # 创建管理器
    manager = MultiIMUManager()
    
    # 添加所有设备
    for address in imu_addresses:
        manager.add_device(
            mac_address=address,
            save_data=True,
            path="imu.csv",
            buffer_size=2000,   # ✅ 增大缓冲区
            batch_size=20       # ✅ 批量写入
        )
    
    manager.print_status()
    
    # 并发连接所有设备
    try:
        await manager.connect_all()
    except KeyboardInterrupt:
        print("⚠️ 程序被中断")
    except Exception as e:
        print(f"❌ 发生错误: {e}")
    finally:
        await manager.disconnect_all()


# 方式2: 先扫描再连接
async def scan_and_connect():
    # 扫描设备
    devices = await BluetoothIMU.scan_devices(timeout=10)
    
    if not devices:
        print("未找到设备")
        return
    
    # 创建管理器
    manager = MultiIMUManager()
    
    # 选择要连接的设备
    selected_devices = devices[:3] if len(devices) >= 3 else devices
    
    for device in selected_devices:
        manager.add_device(
            mac_address=device.address,
            save_data=True,
            buffer_size=2000,
            batch_size=20
        )
    
    manager.print_status()
    
    try:
        await manager.connect_all()
    except KeyboardInterrupt:
        print("⚠️ 程序被中断")
    finally:
        await manager.disconnect_all()


if __name__ == "__main__":
    # 运行主程序
    asyncio.run(main())
    
    # 或者先扫描
    # asyncio.run(scan_and_connect())
