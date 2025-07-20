
# 版本: 2.0.1
# 作者: StanleyChanH
# 描述: 该脚本将 MaixCam Pro 串口数据接收器封装成一个类 (MaixCamReceiver)，
#       方便在其他 Python 项目中作为模块导入和使用。
#       它完整地实现了与 MaixCam Pro 脚本 (v1.9.1) 配套的通信协议，
#       并通过“读取即消费”机制修复了在发送端停止后，接收端会重复打印最后一条数据的问题。

import serial
import struct
import time
from threading import Thread, Lock

class MaixCamReceiver:
    """
    一个用于接收和解析来自 MaixCam Pro 串口数据的类。
    """
    # --- 通信协议定义 ---
    _SOF = b'\xAA\x55'  # 帧起始符
    _EOF = b'\x55\xAA'  # 帧结束符
    _PAYLOAD_FORMAT = '<4HQ' # 4H for bbox, Q for timestamp
    _PAYLOAD_SIZE = struct.calcsize(_PAYLOAD_FORMAT)
    _CHECKSUM_SIZE = 1
    _EXPECTED_PACKET_LEN = _PAYLOAD_SIZE + _CHECKSUM_SIZE
    
    # CRC8 查表法
    _CRC8_TABLE = (
        0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
        0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
        0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
        0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
        0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
        0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
        0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
        0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
        0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
        0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
        0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
        0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
        0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
        0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
        0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
        0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
    )

    def __init__(self, port, baudrate, debug=False):
        """
        初始化接收器。
        :param port: 串口设备路径 (例如 '/dev/ttyAMA0')
        :param baudrate: 波特率 (例如 115200)
        :param debug: 是否打印调试信息
        """
        self.port = port
        self.baudrate = baudrate
        self.debug = debug
        self.ser = None
        self.is_running = False
        self.thread = None
        self.lock = Lock()
        self.latest_data = None
        
    def _calculate_crc8(self, data_bytes):
        """为给定的数据计算 CRC-8 校验和。"""
        crc = 0
        for byte in data_bytes:
            crc = self._CRC8_TABLE[crc ^ byte]
        return crc

    def start(self):
        """打开串口并启动后台接收线程。"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.is_running = True
            self.thread = Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            if self.debug:
                print(f"[信息] 成功打开串口 {self.port}，接收线程已启动。")
            return True
        except serial.SerialException as e:
            print(f"[致命错误] 无法打开串口 {self.port}: {e}")
            return False

    def stop(self):
        """停止接收线程并关闭串口。"""
        self.is_running = False
        if self.thread:
            self.thread.join() # 等待线程结束
        if self.ser and self.ser.is_open:
            self.ser.close()
            if self.debug:
                print("[信息] 接收线程已停止，串口已关闭。")

    def get_latest_data(self):
        """
        获取并消费最新接收到的数据。
        调用此方法后，内部数据将被清除，直到接收到下一个数据包。
        :return: 最新的数据字典，或 None (如果没有新数据)。
        """
        with self.lock:
            data_to_return = self.latest_data
            self.latest_data = None  # 读取后即消费数据
            return data_to_return

    def _read_loop(self):
        """在后台线程中运行的循环，用于持续接收和解析数据。"""
        sync_state = 0
        
        while self.is_running:
            try:
                # 1. 寻找帧头 (SOF)
                if sync_state == 0:
                    byte = self.ser.read(1)
                    if byte == self._SOF[:1]: # 读到 0xAA
                        sync_state = 1
                elif sync_state == 1:
                    byte = self.ser.read(1)
                    if byte == self._SOF[1:]: # 读到 0x55
                        sync_state = 2
                    else:
                        sync_state = 0 # 不是完整的帧头，重置状态
                
                # 2. 如果找到帧头，开始接收数据包
                if sync_state == 2:
                    # 读取数据包长度 (1字节)
                    len_byte = self.ser.read(1)
                    if not len_byte:
                        sync_state = 0
                        continue
                    
                    packet_len = len_byte[0]
                    if packet_len != self._EXPECTED_PACKET_LEN:
                        if self.debug:
                            print(f"[警告] 数据包长度不匹配。预期: {self._EXPECTED_PACKET_LEN}, 收到: {packet_len}。")
                        sync_state = 0
                        continue

                    # 读取载荷、校验和、帧尾
                    full_packet_data = self.ser.read(packet_len + len(self._EOF))
                    
                    # 校验数据完整性
                    if len(full_packet_data) != (packet_len + len(self._EOF)):
                        if self.debug:
                            print("[警告] 读取数据包超时，数据不完整。")
                        sync_state = 0
                        continue
                    
                    # 提取各部分
                    packet_data = full_packet_data[:packet_len]
                    eof_data = full_packet_data[packet_len:]

                    # 校验帧尾
                    if eof_data != self._EOF:
                        if self.debug:
                            print("[警告] 帧尾不匹配。")
                        sync_state = 0
                        continue
                    
                    # 3. 校验和验证
                    payload_bytes = packet_data[:-self._CHECKSUM_SIZE]
                    received_checksum = packet_data[-self._CHECKSUM_SIZE]
                    calculated_checksum = self._calculate_crc8(payload_bytes)
                    
                    if received_checksum == calculated_checksum:
                        # 校验成功，解析并存储数据
                        unpacked_data = struct.unpack(self._PAYLOAD_FORMAT, payload_bytes)
                        with self.lock:
                            self.latest_data = {
                                'x': unpacked_data[0],
                                'y': unpacked_data[1],
                                'w': unpacked_data[2],
                                'h': unpacked_data[3],
                                'timestamp': unpacked_data[4]
                            }
                    else:
                        if self.debug:
                            print(f"[错误] CRC8校验失败! 收到: {hex(received_checksum)}, 计算出: {hex(calculated_checksum)}")

                    # 一个数据包处理完毕，重置状态
                    sync_state = 0
            
            except serial.SerialException:
                print("[错误] 串口断开连接。正在尝试关闭...")
                self.is_running = False
            except Exception as e:
                print(f"[错误] 接收线程发生未知错误: {e}")


# =============================================================================
# --- 示例用法 ---
# =============================================================================
if __name__ == "__main__":
    # --- 配置 ---
    # 串口设备路径
    SERIAL_PORT = '/dev/ttyAMA4'
    # 波特率
    BAUDRATE = 115200

    print("--- MaixCam 串口接收器测试程序 ---")
    print(f"将从 {SERIAL_PORT} 以 {BAUDRATE} 波特率接收数据。")
    print("按 Ctrl+C 退出程序。")
    
    # 初始化接收器
    receiver = MaixCamReceiver(port=SERIAL_PORT, baudrate=BAUDRATE, debug=True)
    
    if receiver.start():
        try:
            while True:
                data = receiver.get_latest_data()
                if data:
                    print(f"最新数据: X={data['x']}, Y={data['y']}, W={data['w']}, H={data['h']}, Time={data['timestamp']}")
                
                # 主程序可以做其他事情，这里只做简单延时
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n[信息] 正在关闭程序...")
        finally:
            receiver.stop()
    else:
        print("[错误] 无法启动接收器，程序退出。")
