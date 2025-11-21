import serial
import threading
import time
import struct
import os

# --- 发送协议常量 (30字节) ---
FRAME_HEAD_1 = 0x55
FRAME_HEAD_2 = 0xAA
FRAME_LEN_SEND = 0x1E      # 发送长度 30
CMD_SEND = 0x01

class CANDriver:
    def __init__(self, port="/dev/ttyACM0", baudrate=921600):
        os.system(f"sudo chmod 777 {port}")
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.05)
            print(f"[CAN Driver] 串口 {port} 打开成功")
        except Exception as e:
            print(f"[CAN Driver] 错误: {e}")
            self.ser = None

        self.running = True
        self.rx_callback = None
        
        if self.ser and self.ser.isOpen():
            self.thread = threading.Thread(target=self._rx_thread)
            self.thread.daemon = True
            self.thread.start()

    def set_rx_callback(self, callback):
        self.rx_callback = callback

    def send_can_frame(self, can_id, data_bytes):
        """发送 30 字节标准帧"""
        if not self.ser: return
        if len(data_bytes) != 8: return

        buffer = bytearray(30)
        buffer[0] = FRAME_HEAD_1
        buffer[1] = FRAME_HEAD_2
        buffer[2] = FRAME_LEN_SEND
        buffer[3] = CMD_SEND
        buffer[4:8] = (1).to_bytes(4, 'little') # Count
        buffer[8:12] = (0).to_bytes(4, 'little') # Interval
        buffer[12] = 0 # Std Frame
        buffer[13:17] = int(can_id).to_bytes(4, 'little') # ID
        buffer[17] = 0 # Data Frame
        buffer[18] = 8 # DLC
        
        for i in range(8):
            buffer[21+i] = data_bytes[i]
            
        buffer[29] = 0x88 # 校验/尾部
        try:
            self.ser.write(buffer)
        except:
            pass

    def _rx_thread(self):
        """接收处理：解析 17 字节的回传帧"""
        buffer = bytearray()
        while self.running and self.ser:
            try:
                if self.ser.inWaiting() > 0:
                    new_data = self.ser.read(self.ser.inWaiting())
                    buffer.extend(new_data)
                    
                    # 循环解析
                    while len(buffer) >= 17: # 接收帧只有 17 字节
                        # 寻找帧头 55 AA
                        if buffer[0] == 0x55 and buffer[1] == 0xAA:
                            frame_len = buffer[2] # 应该是 0x11 (17)
                            
                            # 确保缓冲区够长
                            if len(buffer) < frame_len:
                                break 
                            
                            # 提取一帧
                            frame = buffer[:frame_len]
                            buffer = buffer[frame_len:]
                            
                            # --- 关键修正：提取 ID 和 数据 ---
                            # ID 在 frame[4:8]
                            can_id = int.from_bytes(frame[4:8], 'little')
                            
                            # 数据在 frame[8:16] (共8字节)
                            payload = frame[8:16]
                            
                            if self.rx_callback:
                                self.rx_callback(can_id, payload)
                        else:
                            # 没找到头，滑动一个字节
                            buffer.pop(0)
                            
                else:
                    time.sleep(0.005)
            except Exception as e:
                print(f"Rx Error: {e}")
                buffer = bytearray() # 清空防止死循环