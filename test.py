import asyncio
from bleak import BleakScanner, BleakClient
from datetime import datetime 
import struct
from queue import Queue
import csv
import os


class BluetoothIMU:
        def __init__(self, mac_address, path = 'imu.csv', save_data = False, data_len = 41):
            self.mac_address = mac_address
            self.sock = None
            self.data_queue = Queue()
            self.running = False
            self.START_FLAG = b'\x3A'  # 包头: 0x3A
            self.END_FLAG = b'\x0D\x0A'  # 包尾: 0x0D0A (即 '\r\n')
            self.csv_file_path = self.getUniqueFilename(path)
            self.data_len = data_len  # 每个数据包的长度为41字节
            self.buffer = asyncio.Queue()
            self.saveflag = save_data  # 是否保存数据到CSV文件的标志
            self.step = 0
            self.raw_buffer = bytearray()
            self.unpacked_data = []


        async def scan_devices(timeout=5):
            """扫描 BLE 设备"""
            print(f"🔍 正在扫描 BLE 设备 ({timeout}s)...")
            devices = await BleakScanner.discover(timeout=timeout)
            if not devices:
                print("❌ 没有发现设备")
                return []
            for i, d in enumerate(devices):
                print(f"{i}: {d.name or '未知设备'} [{d.address}]")
            return devices

        def process_data(self, sender, data):
            try:
                self.buffer.put_nowait(data)
            except Exception as e:
                print("⚠️ 数据入队失败:", e)


            # ✅ 数据处理任务：持续从队列取数据、解析、打印/保存
        async def data_handler(self):
            print("🧩 数据处理线程已启动")
            while True:
                data = await self.buffer.get()  # 等待新数据（不会阻塞事件循环）
                await self.handle_packet(data)

        async def handle_packet(self, data):
            self.raw_buffer.extend(data)
            while len(self.raw_buffer) > self.data_len:
            # 查找帧头
                start_index = self.raw_buffer.find(self.START_FLAG)

                if start_index == -1:
                    # 没有找到帧头，清空缓冲区
                    if len(self.raw_buffer) > 0:
                        print(f"❌ 未找到帧头 0x3A，丢弃数据: {self.raw_buffer.hex()}")
                    self.raw_buffer.clear()
                    break
                
                # 如果帧头不在开始位置，丢弃之前的数据
                if start_index > 0:
                    discarded = self.raw_buffer[:start_index]
                    print(f"⚠️  丢弃帧头前的数据: {discarded.hex()}")
                    self.raw_buffer = self.raw_buffer[start_index:]

                # 查找帧尾（从帧头之后开始查找）
                end_index = self.raw_buffer.find(self.END_FLAG, 1)

                if end_index == -1:
                    # 没有找到完整的帧尾，等待更多数据
                    print(f"⏳ 等待更多数据... (当前缓冲区: {len(self.raw_buffer)} 字节)")

                    # 如果缓冲区过大，可能是数据损坏
                    if len(self.raw_buffer) > 1024:
                        print(f"❌ 缓冲区溢出，清空数据")
                        self.raw_buffer.clear()
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
                    if self.unpacked_data and self.saveflag == True:
                        # 保存到CSV（如果需要）
                        self.save_data_to_csv(self.unpacked_data)
                else:
                    print(f"❌ 数据包格式错误: {packet.hex()}\n")
        
        def save_data_to_csv(self, data):
            t = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            file_exists = os.path.isfile(self.csv_file_path)
            with open(self.csv_file_path, mode='a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                if not file_exists:
                    # 写入表头
                    writer.writerow([
                        "Timestamp", "imuTimestamp","Gyro_X (rad/s)", "Gyro_Y (rad/s)", "Gyro_Z (rad/s)",
                        "Acc_X (g)", "Acc_Y (g)", "Acc_Z (g)",
                        "Mag_X (μT)", "Mag_Y (μT)", "Mag_Z (μT)",
                        "Euler_X", "Euler_Y", "Euler_Z",
                        "Quaternion_W", "Quaternion_X", "Quaternion_Y", "Quaternion_Z",
                        "Linear_Acc_X (g)", "Linear_Acc_Y (g)", "Linear_Acc_Z (g)"
                    ])
                # 写入数据行
                writer.writerow([
                    t,
                    data["timestamp"],
                    *data["gyro"],
                    *data["acc"],
                    *data["mag"],
                    *data["euler"],
                    *data["quaternion"],
                    *data["linear_acceleration"],
                ])

        def unpack_data(self, data):
            # 提取Sensor ID
            sensor_id = struct.unpack('<H', data[1:3])[0]
            # print(f"Sensor ID: {sensor_id}")

            # 提取指令号
            command_id = struct.unpack('<H', data[3:5])[0]
            # print(f"Command ID: {command_id}")

            # 提取数据长度
            data_length = struct.unpack('<H', data[5:7])[0]
            # print(f"Data Length: {data_length} bytes")

            # 提取时间戳
            timestamp_sec = struct.unpack('<I', data[7:11])[0] / 400
            # print(f"{timestamp_sec:.4f} seconds")

            # 提取传感器数据，从数据包索引11开始，长度为44字节
            sensor_data = data[11:-4]  # 从数据包索引11开始，直到索引-4结束（44字节数据）
            # print(f"Sensor Data Len: {len(sensor_data)} bytes")

            # 示例提取陀螺仪、加速度计、磁力计数据
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

            # ✅ 蓝牙连接与订阅
        async def connect_and_read(self):
            print(f"🔗 尝试连接 {self.mac_address} ...")
            async with BleakClient(self.mac_address) as client:
                if not await client.is_connected():
                    print("❌ 连接失败")
                    return
                print("✅ 连接成功！")

                # 寻找可通知特征
                readable_chars = [
                    c for service in client.services for c in service.characteristics
                    if "notify" in c.properties
                ]
                if not readable_chars:
                    print("⚠️ 未发现可通知特征")
                    return

                char = readable_chars[0]
                print(f"🔔 订阅 {char.uuid} 的通知...")

                await client.start_notify(char.uuid, self.process_data)

                # ✅ 同时运行数据处理任务
                await self.data_handler()  # 异步处理数

        async def wait_forever(self):
            """保持事件循环运行"""
            while True:
                await asyncio.sleep(10)
                    
        def getUniqueFilename(self, path):
            base, ext = os.path.splitext(path)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            return f"{base}_{timestamp}{ext}"
        
        def disconnect(self):
            """断开连接"""
            print(f"🔌 断开与 {self.mac_address} 的连接...")
            self.running = False
            
# ===== 多设备管理器 =====
class MultiIMUManager:
    def __init__(self):
        self.imu_devices = []
        self.tasks = []
    
    def add_device(self, mac_address, save_data=True, path=None):
        """添加IMU设备"""
        if path is None:
            path = f"imu_{mac_address.replace(':', '')}.csv"
        
        imu = BluetoothIMU(
            mac_address=mac_address,
            save_data=save_data,
            path=path
        )
        self.imu_devices.append(imu)
        print(f"➕ 添加设备: {mac_address}")
        return imu
    
    async def connect_all(self):
        """并发连接所有设备"""
        print(f"🚀 开始并发连接 {len(self.imu_devices)} 个设备...")
        
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
        print("🔌 断开所有设备...")
        disconnect_tasks = [imu.disconnect() for imu in self.imu_devices]
        await asyncio.gather(*disconnect_tasks, return_exceptions=True)
        print("✅ 所有设备已断开")
    


# ===== 使用示例 =====
async def main():
    # 方式1: 手动指定多个设备地址
    imu_addresses = [
        "00:04:3E:6C:51:C1",
        "00:04:3E:86:27:F0",  # 添加更多设备
        "00:04:3E:86:27:ED",
    ]
    
    # 创建管理器
    manager = MultiIMUManager()
    
    # 添加所有设备
    for i, address in enumerate(imu_addresses):
        manager.add_device(
            mac_address=address,
            save_data=True,
            path=f"imu_{i}.csv"
        )
    
    
    # 并发连接所有设备
    try:
        await manager.connect_all()
    except KeyboardInterrupt:
        print("⚠️ 程序被中断")
    finally:
        await manager.disconnect_all()


if __name__ == "__main__":
    asyncio.run(main())
