import bluetooth
import struct
import time
import threading
from queue import Queue
import csv

# 包头和包尾的定义
START_FLAG = b'\x3A'  # 包头: 0x3A
END_FLAG = b'\x0D\x0A'  # 包尾: 0x0D0A (即 '\r\n')

# CSV文件路径
csv_file_path = "D:/IMU_PC21/sensor_data2.csv"

# 创建CSV文件并写入表头
def create_csv():
    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            "Timestamp", "Gyro_X", "Gyro_Y", "Gyro_Z",
            "Acc_X", "Acc_Y", "Acc_Z", "Mag_X", "Mag_Y", "Mag_Z",
            "Euler_X", "Euler_Y", "Euler_Z",
            "Quaternion_W", "Quaternion_X", "Quaternion_Y", "Quaternion_Z",
            "Linear_Acc_X", "Linear_Acc_Y", "Linear_Acc_Z"
        ])

# 计算校验和函数（假设使用简单的和校验）
def calculate_checksum(data):
    return sum(data) % 256  # 返回校验和，取低8位

# 解包函数：根据LPBUS协议提取数据
def unpack_data(data):
    # 打印接收到的原始数据，以十六进制格式显示
    print(f"Received raw data (hex): {data.hex()}")

    # 确保数据包长度为48字节
    if len(data) != 59:
        print(f"Invalid data length: Expected 59 bytes, got {len(data)} bytes.")
        return None

    # 提取Sensor ID
    sensor_id = struct.unpack('<H', data[1:3])[0]
    print(f"Sensor ID: {sensor_id}")

    # 提取指令号
    command_id = struct.unpack('<H', data[3:5])[0]
    print(f"Command ID: {command_id}")

    # 提取数据长度
    data_length = struct.unpack('<H', data[5:7])[0]
    print(f"Data Length: {data_length} bytes")

    # 提取时间戳
    timestamp = struct.unpack('<I', data[7:11])[0]
    timestamp_sec = timestamp / 400  # 转换为秒
    print(f"Timestamp: {timestamp} (raw), {timestamp_sec:.4f} seconds")

    # 提取传感器数据，从数据包索引11开始，长度为44字节
    sensor_data = data[11:-4]  # 从数据包索引11开始，直到索引-4结束（44字节数据）
    print(f"Sensor Data: {sensor_data.hex()}")

    # 示例提取陀螺仪、加速度计、磁力计数据（假设它们的顺序按协议排列）
    gyro_x = struct.unpack('<h', sensor_data[0:2])[0] * 1e-3  # 陀螺仪X轴数据，单位 rad/s
    gyro_y = struct.unpack('<h', sensor_data[2:4])[0] * 1e-3  # 陀螺仪Y轴数据，单位 rad/s
    gyro_z = struct.unpack('<h', sensor_data[4:6])[0] * 1e-3  # 陀螺仪Z轴数据，单位 rad/s

    acc_x = struct.unpack('<h', sensor_data[6:8])[0] * 1e-3  # 加速度计X轴数据，单位 g
    acc_y = struct.unpack('<h', sensor_data[8:10])[0] * 1e-3  # 加速度计Y轴数据，单位 g
    acc_z = struct.unpack('<h', sensor_data[10:12])[0] * 1e-3  # 加速度计Z轴数据，单位 g

    mag_x = struct.unpack('<h', sensor_data[12:14])[0] * 1e-2  # 磁力计X轴数据，单位 μT
    mag_y = struct.unpack('<h', sensor_data[14:16])[0] * 1e-2  # 磁力计Y轴数据，单位 μT
    mag_z = struct.unpack('<h', sensor_data[16:18])[0] * 1e-2  # 磁力计Z轴数据，单位 μT

    eulerx = struct.unpack('<h', sensor_data[32:34])[0] * 1e-4
    eulery = struct.unpack('<h', sensor_data[34:36])[0] * 1e-4
    eulerz = struct.unpack('<h', sensor_data[36:38])[0] * 1e-4

    quaternionw = struct.unpack('<h', sensor_data[24:26])[0] * 1e-4
    quaternionx = struct.unpack('<h', sensor_data[26:28])[0] * 1e-4
    quaterniony = struct.unpack('<h', sensor_data[28:30])[0] * 1e-4
    quaternionz = struct.unpack('<h', sensor_data[30:32])[0] * 1e-4

    linear_accx = struct.unpack('<h', sensor_data[38:40])[0] * 1e-3
    linear_accy = struct.unpack('<h', sensor_data[40:42])[0] * 1e-3
    linear_accz = struct.unpack('<h', sensor_data[42:44])[0] * 1e-3

    print(f"Gyro Data: X={gyro_x} rad/s, Y={gyro_y} rad/s, Z={gyro_z} rad/s")
    print(f"Acc Data: X={acc_x} g, Y={acc_y} g, Z={acc_z} g")
    print(f"Mag Data: X={mag_x} μT, Y={mag_y} μT, Z={mag_z} μT")
    print(f"euler: x={eulerx} rad, y={eulery} rad, z={eulerz} rad")
    print(f"quaternionq: w={quaternionw}, x={quaternionx}, y={quaterniony}, z={quaternionz}")
    print(f"linear_accx: x={linear_accx} g, y={linear_accy} g, z={linear_accz} g")

    # 将数据保存到 CSV 文件
    with open(csv_file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            timestamp_sec, gyro_x, gyro_y, gyro_z,
            acc_x, acc_y, acc_z, mag_x, mag_y, mag_z,
            eulerx, eulery, eulerz,
            quaternionw, quaternionx, quaterniony, quaternionz,
            linear_accx, linear_accy, linear_accz
        ])

    return {
        "gyro": (gyro_x, gyro_y, gyro_z),
        "acc": (acc_x, acc_y, acc_z),
        "mag": (mag_x, mag_y, mag_z),
        "euler": (eulerx, eulery, eulerz),
        "quaternion": (quaternionw, quaternionx, quaterniony, quaternionz),
        "linear_acceleration": (linear_accx, linear_accy, linear_accz),
    }

# 蓝牙连接函数
def connect_to_imu(target_address):
    port = 1  # 蓝牙通常使用RFCOMM端口1
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((target_address, port))
    print(f"Connected to IMU at {target_address}")
    return sock

# 数据接收线程
def receive_data(sock, stop_event, buffer):
    while not stop_event.is_set():
        try:
            data = sock.recv(1024)  # 接收数据
            if data:
                buffer.append(data)  # 缓存接收到的数据

                # 尝试合并缓存的数据包
                while len(buffer) > 0:
                    combined_data = b''.join(buffer)  # 合并所有缓存的数据
                    if len(combined_data) >= 59:
                        # 检查数据包是否完整
                        if combined_data[:1] != START_FLAG:
                            print("Invalid start flag!")
                            buffer.clear()  # 清空缓存
                            break
                        if combined_data[57:59] != END_FLAG:
                            print("Invalid end flag!")
                            buffer.clear()  # 清空缓存
                            break
                        # 提取有效数据
                        packet = combined_data[:59]  # 获取完整的59字节数据包
                        print(f"Extracted packet: {packet.hex()}")
                        # 解包并处理
                        unpacked_data = unpack_data(packet)
                        if unpacked_data:
                            print(f"Processed data: {unpacked_data}")

                        # 移除已处理的数据
                        remaining_data = combined_data[59:]
                        buffer.clear()
                        if len(remaining_data) > 0:
                            buffer.append(remaining_data)  # 将剩余数据放回缓冲区
                    else:
                        break  # 如果缓冲区还没有足够的数据，不处理

        except bluetooth.BluetoothError as e:
            print(f"Bluetooth error: {e}")
        except Exception as e:
            print(f"Error in receive thread: {e}")
            break

# 主程序：连接IMU并启动数据接收线程
def main():
    target_address = "00:04:3E:86:27:ED"  # 替换为IMU设备的蓝牙MAC地址

    # 创建CSV文件
    create_csv()

    # 连接IMU设备
    sock = connect_to_imu(target_address)

    # 创建缓冲区
    buffer = []

    stop_event = threading.Event()  # 停止事件标志

    # 创建并启动接收数据的线程
    receive_thread = threading.Thread(target=receive_data, args=(sock, stop_event, buffer))
    receive_thread.start()

    # 主线程可以继续执行其他任务
    try:
        while True:
            time.sleep(1)  # 主线程执行任务的地方
    except KeyboardInterrupt:
        print("Exiting main program...")
    finally:
        stop_event.set()  # 设置停止标志，终止接收线程
        sock.close()  # 关闭套接字
        receive_thread.join()  # 确保线程完成后退出

if __name__ == "__main__":
    main()
