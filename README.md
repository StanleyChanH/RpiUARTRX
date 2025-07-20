# MaixCam 串口数据接收器 (MaixCamReceiver)
这是一个运行在树莓派（或其他 Linux 主机）上的 Python 模块，用于接收和解析来自 MaixCam Pro 的目标检测数据。它被封装成一个易于使用的 MaixCamReceiver 类，可以方便地集成到您的其他项目中。

该模块完整地实现了与配套的 MaixCam Pro 脚本 (v1.9.1) 的通信协议，确保了数据接收的稳定性和准确性。

## 主要功能
完整的协议实现: 严格遵循 帧头 (SOF) + 长度 + 载荷 + 校验和 (CRC8) + 帧尾 (EOF) 的数据帧结构。

健壮的数据校验: 对每一帧数据进行长度和 CRC8 校验，自动丢弃损坏或不完整的数据包。

多线程设计: 在后台独立线程中进行数据接收和解析，不会阻塞您的主程序逻辑。

易于使用的接口: 只需实例化类，调用 start()、get_latest_data() 和 stop() 方法即可轻松使用。

“读取即消费”机制: get_latest_data() 方法在获取数据后会自动清除缓存，避免重复处理陈旧数据。

## 硬件与软件要求
### 硬件
树莓派 (Raspberry Pi): 推荐树莓派3B+及以上型号。

MaixCam Pro: 运行配套的串口发送脚本 (kalman_maix_pro_cn.py v1.9.1 或更高版本)。

### 软件

Python 3

pyserial 库: 如果尚未安装，请使用以下命令安装：

```bash
uv sync
# uv add pyserial
```

### 如何使用
1. 硬件连接
    请确保 MaixCam Pro 和树莓派已正确连接：

    MaixCam TX -> Raspberry Pi RX (GPIO 15)

    MaixCam RX -> Raspberry Pi TX (GPIO 14)

    MaixCam GND -> Raspberry Pi GND (地线必须连接！)

2. 配置树莓派串口
您需要启用树莓派的硬件串口，并禁用串口登录功能。

    运行 sudo raspi-config。

    选择 3 Interface Options -> I6 Serial Port。

    回答 <No> (是否需要一个登录 shell)。

    回答 <Yes> (是否启用串口硬件)。

    重启树莓派使配置生效。

3. 在您的项目中使用
将 rpi5_uart_receiver.py 文件与您的主项目文件放在同一个目录下。然后，您可以像下面这样导入并使用它：
```python
# 导入您的主程序，例如: main_control.py

import time
from rpi5_uart_receiver import MaixCamReceiver

# --- 配置 ---
SERIAL_PORT = '/dev/ttyAMA0'  # 树莓派硬件串口
BAUDRATE = 115200

# 1. 初始化接收器
#    将 debug 设置为 True 会在控制台打印详细的接收日志
receiver = MaixCamReceiver(port=SERIAL_PORT, baudrate=BAUDRATE, debug=False)

# 2. 启动接收器
#    这将打开串口并在后台启动一个线程来监听数据
if not receiver.start():
    print("无法启动接收器，程序退出。")
    exit()

print("接收器已启动。按 Ctrl+C 退出。")

# 3. 在主循环中获取数据
try:
    while True:
        # get_latest_data() 会返回最新接收到的数据，或者 None
        # 数据在被读取一次后就会被清除
        maix_data = receiver.get_latest_data()

        if maix_data:
            # 在这里处理您的逻辑
            print(f"接收到新数据: X={maix_data['x']}, Y={maix_data['y']}")
            # 例如: control_robot(maix_data['x'], maix_data['y'])

        # 您的主程序可以继续执行其他任务
        time.sleep(0.05) # 避免CPU占用过高

except KeyboardInterrupt:
    print("\n程序正在关闭...")

finally:
    # 4. 停止接收器
    #    这将安全地停止后台线程并关闭串口
    receiver.stop()
    print("程序已安全退出。")
```

4. 直接运行测试

您也可以直接运行 main.py 文件来进行快速测试。它会连接到串口并持续打印接收到的数据。
```bash
uv run main.py
```
