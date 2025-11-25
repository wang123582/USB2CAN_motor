import serial
import threading
import time
import os

# === 协议常量 ===
TX_HEAD_1 = 0x55
TX_HEAD_2 = 0xAA
TX_LEN    = 0x1E  # 30字节
TX_CMD    = 0x01

class CANBus:
    """单个 CAN 总线接口 (对应一个物理串口)"""
    def __init__(self, bus_id, port, baudrate=921600):
        self.bus_id = bus_id  # 逻辑 ID (0, 1, 2...)
        self.port = port
        self.ser = None
        self.running = False
        self.lock = threading.Lock()
        
        # 自动提权
        try:
            os.system(f"sudo chmod 777 {port}")
        except:
            pass

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.02)
            print(f"[CAN Bus {bus_id}] 串口 {port} 连接成功")
            self.running = True
        except Exception as e:
            print(f"[CAN Bus {bus_id}] 打开失败: {e}")

    def send_frame(self, can_id, data_bytes):
        """发送单帧 (30字节协议)"""
        if not self.ser or not self.running: return

        buffer = bytearray(30)
        buffer[0] = TX_HEAD_1
        buffer[1] = TX_HEAD_2
        buffer[2] = TX_LEN
        buffer[3] = TX_CMD
        buffer[4] = 0x01 # 次数
        buffer[12] = 0x00 # ID类型
        buffer[13:17] = int(can_id).to_bytes(4, byteorder='little')
        buffer[17] = 0x00 # 数据帧
        buffer[18] = 0x08 # DLC
        
        for i in range(8):
            buffer[21+i] = data_bytes[i]
            
        buffer[29] = 0x88 # 校验

        try:
            with self.lock:
                self.ser.write(buffer)
        except Exception as e:
            print(f"[Bus {self.bus_id}] Tx Error: {e}")

    def close(self):
        self.running = False
        if self.ser:
            self.ser.close()

class CANNetwork:
    """CAN 网络管理器，管理多路总线"""
    def __init__(self):
        self.buses = {} # {bus_id: CANBus_Instance}
        self.rx_callback = None
        self.running = True
        
        # 启动全局接收线程
        self.thread = threading.Thread(target=self._global_rx_thread)
        self.thread.daemon = True
        self.thread.start()

    def add_bus(self, bus_id, port):
        """添加一条总线"""
        new_bus = CANBus(bus_id, port)
        if new_bus.running:
            self.buses[bus_id] = new_bus
        return new_bus

    def set_callback(self, callback):
        """回调格式: callback(bus_id, can_id, data)"""
        self.rx_callback = callback

    def send(self, bus_id, can_id, data):
        """向指定总线发送数据"""
        if bus_id in self.buses:
            self.buses[bus_id].send_frame(can_id, data)

    def _global_rx_thread(self):
        """轮询所有总线的接收数据"""
        buffers = {bid: bytearray() for bid in self.buses}
        
        while self.running:
            data_received = False
            
            # 遍历所有总线
            for bid, bus in list(self.buses.items()):
                if not bus.running or not bus.ser: continue
                
                try:
                    waiting = bus.ser.inWaiting()
                    if waiting > 0:
                        data_received = True
                        chunk = bus.ser.read(waiting)
                        buffers[bid].extend(chunk)
                        
                        # 解析协议 (AA ... 55, 16字节)
                        while len(buffers[bid]) >= 16:
                            if buffers[bid][0] == 0xAA and buffers[bid][15] == 0x55:
                                frame = buffers[bid][:16]
                                buffers[bid] = buffers[bid][16:]
                                
                                can_id = int.from_bytes(frame[3:7], byteorder='little')
                                payload = frame[7:15]
                                
                                if self.rx_callback:
                                    self.rx_callback(bid, can_id, payload)
                            else:
                                buffers[bid].pop(0)
                except Exception as e:
                    print(f"[Net Rx] Bus {bid} Error: {e}")
            
            if not data_received:
                time.sleep(0.002)

    def close_all(self):
        self.running = False
        for bus in self.buses.values():
            bus.close()