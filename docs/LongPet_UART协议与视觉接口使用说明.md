# LongPet 运动 MCU 串口协议与视觉模块接口使用说明

> 适用对象：LongPet 龙芯主控、视觉模块、调试用转接 ESP32  
> 运动 MCU：ESP32-S3  
> 当前协议版本：V2  
> 更新时间：2026-09-11

---

## 1. UART 基本配置

LongPet 运动 MCU 使用 `Serial1` 与龙芯主控通信。

| 项目 | 配置 |
|---|---|
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验 | None |
| 停止位 | 1 |
| 流控 | None |
| RX | GPIO6 |
| TX | GPIO7 |
| 行结束符 | LF 或 CRLF |

初始化：

```cpp
Serial1.begin(115200, SERIAL_8N1, 6, 7);
```

接线必须交叉：

```

主控 TX  ----------> ESP32-S3 GPIO6 RX
主控 RX  <---------- ESP32-S3 GPIO7 TX
主控 GND ---------- ESP32-S3 GND
```

必须共地。

------

## 2. 使用另一块 ESP32 作为 USB-UART 转接器

没有 USB-TTL 时，可以用另一块 ESP32 做透明串口桥。



推荐代码：

```

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 6, 7);
    delay(500);
}

void loop() {
    // PC -> 转接 ESP32 -> LongPet
    while (Serial.available()) {
        Serial1.write(Serial.read());
    }

    // LongPet -> 转接 ESP32 -> PC
    while (Serial1.available()) {
        Serial.write(Serial1.read());
    }
}
```

假设转接 ESP32 也使用 GPIO6/7：

```

转接 ESP32 GPIO7 TX  ---> LongPet GPIO6 RX
转接 ESP32 GPIO6 RX  <--- LongPet GPIO7 TX
GND                   --- GND
```

如果转接板是经典 ESP32-WROOM-32，一般不要使用 GPIO6~11，因为这些 GPIO 通常连接板载 SPI Flash，应改用其他空闲 UART GPIO，例如 16/17。

------

# 3. 协议基本格式

所有指令均为 ASCII 文本，一行一条。



例如：

```

MODE MANUAL
MOVE FORWARD 30
HEAD LEFT 100
STATUS
STOP
```

推荐使用：

```

\r\n
```

作为行尾。



固件同时识别 `\r` 和 `\n`。



当前串口单行缓冲区：

```

64 bytes
```

------

# 4. 控制模式

当前共有四种模式。

## 4.1 SAFE

```

MODE SAFE
```

安全模式。



特点：



-  底盘禁止运动； 
-  上电默认进入 SAFE； 
-  模式切换时会停止当前底盘动作。 

------

## 4.2 HEAD_ONLY

```

MODE HEAD_ONLY
```

视觉头部跟踪模式。



特点：



-  接受 `TARGET`； 
-  可以根据视觉目标控制头部舵机； 
-  底盘被硬禁止运动。 



这是当前视觉联调最推荐使用的模式。

------

## 4.3 MANUAL

```

MODE MANUAL
```

人工控制模式。



允许：

```

MOVE ...
HEAD ...
```

底盘和舵机可以同时控制。

------

## 4.4 FOLLOW

```

MODE FOLLOW
```

为后续自动跟随预留。



当前：



-  接受 `TARGET`； 
-  可以进行头部目标跟踪； 
-  自动底盘跟随仍然关闭。 



因此当前不要把 `FOLLOW` 当成已经实现的自动追人模式。

------

# 5. 通用控制指令

## 5.1 PING

```

PING
```

作用：



-  确认通信链路正常； 
-  刷新 UART 链路超时。 



注意：

```

PING 不会刷新底盘运动租约
```

所以持续发送 PING 不能让车辆一直行驶。

------

## 5.2 STATUS

```

STATUS
```

典型返回：

```

[STATUS] mode=MANUAL motion=STOPPED stop=STOP_COMMAND fault=0 target=0 servo=1570 imu=1
```

字段：

| 字段含义 |                    |
| -------- | ------------------ |
| mode     | 当前模式           |
| motion   | 当前底盘动作       |
| stop     | 最近停车原因       |
| fault    | 是否存在锁存故障   |
| target   | 当前是否有视觉目标 |
| servo    | 舵机脉宽           |
| imu      | IMU 是否正常       |

------

## 5.3 STOP

```

STOP
```

立即停止底盘。



典型返回：

```

[STOP] STOP_COMMAND latched=0
```

上层需要主动停车时应发送 `STOP`，不要只依赖 watchdog。

------

# 6. 底盘运动指令

仅在：

```

MODE MANUAL
```

下有效。



格式：

```

MOVE <方向> <speed>
```

当前速度范围：

```

1 ~ 100
```

这里的 speed 是控制器内部控制量，不直接等于 cm/s。

------

## 前进

```

MOVE FORWARD 30
```

## 后退

```

MOVE BACKWARD 30
```

## 原地左转

```

MOVE ROTATE_LEFT 30
```

## 原地右转

```

MOVE ROTATE_RIGHT 30
```

## 左平移

```

MOVE SHIFT_LEFT 30
```

## 右平移

```

MOVE SHIFT_RIGHT 30
```

当前实机状态：

```

FORWARD       正常
BACKWARD      正常
ROTATE_LEFT   正常
ROTATE_RIGHT  正常
SHIFT_LEFT    存在问题
SHIFT_RIGHT   存在问题
```

由于 LongPet 当前功能暂时不依赖左右平移，因此 `SHIFT_LEFT / SHIFT_RIGHT` 暂缓处理。

------

# 7. MANUAL 运动租约

当前：

```

MANUAL_COMMAND_TIMEOUT = 500 ms
```

也就是说只发送一次：

```

MOVE FORWARD 30
```

车辆最多保持约 500ms。



超过约 500ms 没收到新的 MOVE，会自动停车：

```

[STOP] MANUAL_COMMAND_TIMEOUT latched=0
```

因此龙芯上层需要持续运动时推荐：

```

每 100~200ms 重发一次 MOVE
```

例如：

```

MOVE FORWARD 30
MOVE FORWARD 30
MOVE FORWARD 30
...
```

这是故意设计的安全机制。

------

# 8. 舵机控制

舵机：

```

GPIO2
```

当前参数：

```

中心：1570 us
软件最小：870 us
软件最大：2270 us
默认步长：20 us
单次最大步长：100 us
```

实机验证总机械活动范围约：

```

120°
```

------

## 回中

```

HEAD CENTER
```

回到：

```

1570 us
```

------

## 左转

```

HEAD LEFT 100
```

当前代码：

```

pulse -= 100 us
```

------

## 右转

```

HEAD RIGHT 100
```

当前代码：

```

pulse += 100 us
```

舵机脉宽始终限制在：

```

870 ~ 2270 us
```

------

# 9. 视觉模块需要传递的数据

视觉模块与运动 MCU 之间不需要发送图像，也不需要发送完整 bbox。



只需要：

```

TARGET <dx> <dy> <area>
```

例如：

```

TARGET -32 8 7350
```

三个数据分别为：

```

dx
dy
area
```

------

## 9.1 dx

目标中心相对于画面中心的水平偏差。



推荐定义：

```

dx = target_center_x - image_center_x
```

因此：

```

dx < 0   人在画面左侧
dx = 0   水平居中
dx > 0   人在画面右侧
```

协议允许范围：

```

-4096 ~ 4096
```

目前头部只有水平舵机，因此 `dx` 是最主要的数据。

------

## 9.2 dy

目标中心相对于画面中心的垂直偏差：

```

dy = target_center_y - image_center_y
```

允许范围：

```

-4096 ~ 4096
```

目前暂未用于水平舵机控制，但应该保留，方便以后：



-  增加俯仰舵机； 
-  姿态分析； 
-  更完整的目标控制。 

------

## 9.3 area

目标检测框面积：

```

area = bbox_width * bbox_height
```

单位：

```

pixel²
```

例如：

```

bbox = 80 × 100
area = 8000
```

协议允许范围：

```

0 ~ 16777216
```

未来主要用于粗略判断人物距离。



当前代码暂时保留：

```

far threshold  = 5000
near threshold = 10000
```

但这两个阈值尚未完成摄像头实机标定，因此暂时不能用于正式自动跟随。

------

# 10. 目标丢失

当视觉模块没有检测到可靠目标时发送：

```

TARGET 0 0 0
```

其中：

```

area == 0
```

表示目标丢失。



运动 MCU 会进入目标丢失状态。

------

# 11. TARGET 的使用模式

TARGET 只应在：

```

HEAD_ONLY
```

或者：

```

FOLLOW
```

模式发送。



当前最推荐：

```

MODE HEAD_ONLY
TARGET -20 5 7000
TARGET -15 3 7100
TARGET -8  2 7200
...
```

这样：

```

视觉
 ↓
TARGET
 ↓
舵机跟踪人物
```

同时底盘保持禁止运动。

------

# 12. 旧视觉协议兼容

当前还兼容旧格式：

```

dx dy area
```

例如：

```

-20 5 7000
```

等价于：

```

TARGET -20 5 7000
```

但新代码统一建议使用：

```

TARGET ...
```

避免协议歧义。

------

# 13. 视觉模块 UART 发送架构

当前视觉系统为：

```

Tinyissimo Detector
+
Sparse LK Tracker
```

Detector 较慢，而 tracker 更新更快。



因此不能采用：

```

检测器推理完成
    ↓
发送一次 UART
```

否则检测器运行的 0.7~1.5 秒期间舵机得不到更新。



推荐：

```

Detector --------\
                  \
Tracker ---------> latest_target
                       |
                       v
              UART Publisher
                       |
                       v
               TARGET dx dy area
```

UART Publisher 独立运行。



推荐频率：

```

约 10 Hz
```

即：

```

每约 100ms 发送一次
```

------

# 14. 视觉目标新鲜度

视觉线程应保存：

```

struct VisionTarget {
    bool valid;
    int dx;
    int dy;
    int area;
    uint64_t timestampMs;
};
```

Publisher 判断目标是否仍然新鲜。



示例：

```

if (target.valid &&
    now - target.timestampMs < 300) {

    uart.printf(
        "TARGET %d %d %d\r\n",
        target.dx,
        target.dy,
        target.area
    );

} else {

    uart.print("TARGET 0 0 0\r\n");
}
```

推荐上层 freshness：

```

约 300ms
```

MCU 自身 TARGET timeout：

```

约 500ms
```

因此不允许持续发送已经过期的 tracker 位置。

------

# 15. 推荐工作方式

## 视觉人物跟踪

```

MODE HEAD_ONLY
TARGET dx dy area
TARGET dx dy area
...
```

## 人工遥控

```

MODE MANUAL
MOVE FORWARD 30
MOVE FORWARD 30
...
HEAD LEFT 100
HEAD CENTER
STOP
```

## 安全停车

```

STOP
```

------

# 16. 当前建议上层使用的完整指令集合

```

PING
STATUS
STOP

MODE SAFE
MODE HEAD_ONLY
MODE MANUAL
MODE FOLLOW

TARGET <dx> <dy> <area>

HEAD LEFT <step>
HEAD RIGHT <step>
HEAD CENTER

MOVE FORWARD <speed>
MOVE BACKWARD <speed>
MOVE ROTATE_LEFT <speed>
MOVE ROTATE_RIGHT <speed>
MOVE SHIFT_LEFT <speed>
MOVE SHIFT_RIGHT <speed>
```

其中现阶段正式功能建议只依赖：

```

FORWARD
BACKWARD
ROTATE_LEFT
ROTATE_RIGHT
HEAD
TARGET
STOP
STATUS
```

`SHIFT_LEFT / SHIFT_RIGHT` 当前实机存在问题，暂缓使用。