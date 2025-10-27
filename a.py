# ble_connect_demo.py
import asyncio
from bleak import BleakScanner, BleakClient

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

def callback(sender, data):
    """通知回调函数"""
    print(f"数据: {data}")


async def connect_and_read(address):
    """连接到指定地址的 BLE 设备"""
    print(f"🔗 尝试连接 {address} ...")
    async with BleakClient(address) as client:
        connected = await client.is_connected()
        if not connected:
            print("❌ 连接失败")
            return
        print("✅ 连接成功！")

        # 打印设备所有服务与特征
        print("\n📋 可用服务与特征:")
        for service in client.services:
            print(f"[Service] {service.uuid}")
            for char in service.characteristics:
                props = ",".join(char.properties)
                print(f"   [Char] {char.uuid} ({props})")

        # 尝试读取第一个可读特征
        readable_chars = [c for c in client.services.characteristics.values() if "notify" in c.properties]
        if readable_chars:
            char = readable_chars[0]
            print(f"\n📖 尝试读取 {char.uuid} ...")
            val = await client.read_gatt_char(char.uuid)
            print(f"✅ 读取值: {val}")
            
        else:
            print("⚠️ 设备没有可读特征")

        # 订阅通知

        try:
            if readable_chars:
                char = readable_chars[0]
                print(f"\n🔔 订阅 {char.uuid} 的通知...")
                await client.start_notify(char.uuid, callback)
                while True:
                    await asyncio.sleep(10)
        except Exception as e:
            await client.stop_notify(char.uuid)
            print("✅ 已取消通知订阅")
            print("❌ 订阅通知失败:", e)
        
        # 
        # 示例写入（如设备支持 write）
        # writable_chars = [c for c in client.services.characteristics.values() if "write" in c.properties]
        # if writable_chars:
        #     char = writable_chars[0]
        #     print(f"\n✍️ 尝试向 {char.uuid} 写入数据...")
        #     try:
        #         await client.write_gatt_char(char.uuid, b"hello_ble")
        #         print("✅ 写入成功！")
        #     except Exception as e:
        #         print("❌ 写入失败:", e)

async def main():
    devices = await scan_devices()
    if not devices:
        return

    # === 手动选择目标设备 ===
    # idx = int(input("请输入要连接的设备序号："))
    address = "00:04:3E:6C:52:6A"
    await connect_and_read(address)

if __name__ == "__main__":
    asyncio.run(main())
