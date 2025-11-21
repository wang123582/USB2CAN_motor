import serial
import time
import struct
import sys

# === 配置 ===
PORT = "/dev/ttyACM0"
BAUD = 921600

def print_hex(data, prefix="Raw"):
    print(f"{prefix}: " + " ".join([f"{b:02X}" for b in data]))

def debug_process():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
        print(f"成功打开串口 {PORT}，正在监听数据...")
        print("请用手转动电机，或观察电机上电后的反馈...")
        print("-" * 40)
    except Exception as e:
        print(f"无法打开串口: {e}")
        return

    buffer = bytearray()

    while True:
        try:
            # 读取所有等待的数据
            count = ser.inWaiting()
            if count > 0:
                new_data = ser.read(count)
                buffer.extend(new_data)

                # === 1. 打印原始接收数据 (帮助分析帧头) ===
                # 为了防止刷屏太快，每次打印一部分
                print_hex(new_data, prefix="RECV")

                # === 2. 尝试分析帧结构 ===
                # 只有当缓冲区有一定数据时才分析
                while len(buffer) >= 30: # 假设最小包长是十几字节，取30安全
                    
                    # 🔍 猜测 A: 按照 CSV 的 55 AA 格式寻找
                    if buffer[0] == 0x55 and buffer[1] == 0xAA:
                        # CSV 格式: 55 AA Len Cmd ... Data在第21字节
                        frame_len = buffer[2] # 通常是 0x1E (30)
                        
                        if len(buffer) < frame_len:
                            break # 数据不够，等待下一波
                        
                        # 提取一整帧
                        frame = buffer[:frame_len]
                        buffer = buffer[frame_len:] # 移出缓冲区
                        
                        print(f"发现 55 AA 帧 (Len={frame_len})")
                        
                        # 提取 Payload (根据CSV是 Data21-Data28)
                        # 索引 21 到 29
                        payload = frame[21:29]
                        print_hex(payload, prefix="  >> Payload")
                        
                        # 尝试解析 DJI 数据
                        try_parse_dji(payload)

                    # 🔍 猜测 B: 按照你旧代码的 0x64 0x63 (d c) 格式寻找
                    elif buffer[0] == 100 and buffer[1] == 99:
                        # 旧代码逻辑不明，假设紧跟的是数据
                        # 假设包长 20 左右？这里仅做演示
                        print("发现旧协议头 (64 63)!")
                        # 假设后面紧跟ID和数据，先把前几字节打出来看看
                        print_hex(buffer[:16], prefix="  >> OldProto")
                        buffer = buffer[1:] # 移一位继续找
                        
                    else:
                        # 如果都不是，滑动窗口，丢弃第一个字节，继续找
                        buffer.pop(0)

            time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n停止调试")
            break
        except Exception as e:
            print(f"错误: {e}")
            break

def try_parse_dji(payload):
    """
    尝试将8字节解析为 DJI 电机反馈
    格式: Angle(2) RPM(2) Current(2) Temp(1) Null(1)
    """
    if len(payload) < 8: return
    
    # 使用大端解析
    angle, rpm, current, temp = struct.unpack('>hhhB', payload[:7])
    
    print(f"  >> 解析尝试: RPM={rpm}, Cur={current}, Angle={angle}, Temp={temp}°C")
    
    # 简单的校验逻辑：温度通常在 20-60 之间
    if 10 < temp < 80:
        print("     ✅ 看起来像是正确的数据！")
    else:
        print("     ❌ 数据看起来不对 (温度异常)")

if __name__ == "__main__":
    debug_process()