import bt_connect as btc
import MCU2PC as m2p
import time

def main():
    manager = btc.MultiIMUManager()
    receiver = m2p.SerialReceiver(port="COM19", baudrate=460800, output_csv="joint.csv")
    receiver.start()
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



if __name__ == "__main__":
    main()

