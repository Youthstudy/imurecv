import bluetooth
import struct
import time
import csv

# 包头和包尾的定义
START_FLAG = b'\x3A'  # 包头: 0x3A
END_FLAG = b'\x0D\x0A'  # 包尾: 0x0D0A (即 '\r\n')

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

    return {
        "gyro": (gyro_x, gyro_y, gyro_z),
        "acc": (acc_x, acc_y, acc_z),
        "mag": (mag_x, mag_y, mag_z),
        "euler": (eulerx, eulery, eulerz),
        "quaternion": (quaternionw, quaternionx, quaterniony, quaternionz),
        "linear_acceleration": (linear_accx, linear_accy, linear_accz),
        "timestamp": timestamp_sec  # 包含时间戳数据
    }

# 数据保存到CSV文件的函数
def save_data_to_csv(data, filename="sensor_data.csv"):
    # 字段名称
    fieldnames = ["timestamp", "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z", 
                  "mag_x", "mag_y", "mag_z", "euler_x", "euler_y", "euler_z", 
                  "quaternion_w", "quaternion_x", "quaternion_y", "quaternion_z", 
                  "linear_accx", "linear_accy", "linear_accz"]
    
    # 如果CSV文件不存在，创建文件并写入字段名
    try:
        with open(filename, mode='a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            
            # 写入数据
            writer.writerow({
                "timestamp": data["timestamp"],
                "gyro_x": data["gyro"][0],
                "gyro_y": data["gyro"][1],
                "gyro_z": data["gyro"][2],
                "acc_x": data["acc"][0],
                "acc_y": data["acc"][1],
                "acc_z": data["acc"][2],
                "mag_x": data["mag"][0],
                "mag_y": data["mag"][1],
                "mag_z": data["mag"][2],
                "euler_x": data["euler"][0],
                "euler_y": data["euler"][1],
                "euler_z": data["euler"][2],
                "quaternion_w": data["quaternion"][0],
                "quaternion_x": data["quaternion"][1],
                "quaternion_y": data["quaternion"][2],
                "quaternion_z": data["quaternion"][3],
                "linear_accx": data["linear_acceleration"][0],
                "linear_accy": data["linear_acceleration"][1],
                "linear_accz": data["linear_acceleration"][2]
            })
    except Exception as e:
        print(f"Error saving data to CSV: {e}")

# 蓝牙连接函数
def connect_to_imu(target_address):
    port = 1  # 蓝牙通常使用RFCOMM端口1
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((target_address, port))
    print(f"Connected to IMU at {target_address}") 
    return sock

# 主程序：持续接收IMU数据并处理
def main():
    target_address = "00:04:3E:6C:52:6A"  # 替换为IMU设备的蓝牙MAC地址

    # 连接IMU设备
    sock = connect_to_imu(target_address)

    # 缓存接收到的数据
    buffer = b""

    # 持续接收数据
    while True:
        try:
            data = sock.recv(1024)  # 假设数据包不超过1024字节
            buffer += data  # 将接收到的数据追加到缓存中
            print(f"Received data: {buffer.hex()}")

            # 检查缓存中的数据包，确保每次提取 59 字节（包含包头和包尾）
            while len(buffer) >= 59:
                # 检查包头
                if buffer[:1] != START_FLAG:
                    print("Invalid start flag!")
                    buffer = buffer[1:]  # 丢弃一个字节，继续检查
                    continue

                # 检查包尾
                if buffer[57:59] != END_FLAG:
                    print("Invalid end flag!")
                    buffer = buffer[1:]  # 丢弃一个字节，继续检查
                    continue

                # 提取有效数据
                packet = buffer[0:59]  # 去掉包头和包尾
                print(f"Extracted packet: {packet.hex()}")

                # 解包并处理
                unpacked_data = unpack_data(packet)
                if unpacked_data:
                    print(f"Processed data: {unpacked_data}")
                    # 将数据保存到CSV
                    save_data_to_csv(unpacked_data, filename="D:/IMUshuju/sensor_data3.csv")


                # 从缓存中移除已处理的数据包
                buffer = buffer[59:]

            time.sleep(0.0025)  # 控制接收数据的频率（可以根据需要调整）

        except KeyboardInterrupt:
            print("Exiting...")
            break

    sock.close()

if __name__ == "__main__":
    main()
