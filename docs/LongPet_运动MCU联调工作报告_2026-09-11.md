
# LongPet 运动 MCU 联调与 MPU6500 兼容工作报告

> 项目：LongPet  
> 运动 MCU：ESP32-S3  
> 日期：2026-09-11

---

## 1. 本轮工作内容

本轮主要完成：

1. 排查运动 MCU 的 IMU 初始化失败；
2. 确认实际 IMU 芯片类型；
3. 添加 MPU6500 兼容代码；
4. 实机验证 IMU；
5. 排查 UART 只能发送不能接收的问题；
6. 使用另一块 ESP32 临时作为 USB-UART 转接器；
7. 打通双向 UART；
8. 测试底盘运动；
9. 测试头部舵机；
10. 为后续龙芯主控和视觉模块接入准备协议。

---

# 2. 初始 IMU 故障

最初启动日志：

```text
[STOP] POWER_ON latched=0
[BOOT] SERVO_ATTACH_RESULT=0 pin=2
[STOP] IMU_INIT_FAILED latched=1
```

表现为：



-  I2C 可以扫描到设备； 
-  原 `Adafruit_MPU6050` 初始化失败； 
-  MCU 锁存 `IMU_INIT_FAILED`； 
-  运动安全逻辑因此阻止底盘运行。 

------

# 3. IMU 芯片识别

通过直接读取寄存器得到：

```

WHO_AM_I 0x75 = 0x70
PWR_MGMT_1 0x6B = 0x01
```

I2C 地址：

```

0x68
```

需要区分：

```

0x68 = I2C 总线地址
0x70 = WHO_AM_I 芯片身份
```

由此确认当前器件应按照：

```

MPU6500-compatible
```

处理。



原有 `Adafruit_MPU6050::begin()` 会检查设备身份，因此即使总线通信正常也会初始化失败。

------

# 4. MPU6500 兼容修改

增加：

```

enum class ImuType : uint8_t {
    Unknown,
    MPU6050,
    MPU6500
};
```

初始化阶段读取：

```

WHO_AM_I = 0x75
```

根据结果分流：

```

0x68 -> MPU6050
0x70 -> MPU6500
其他 -> unsupported
```

对于 MPU6050，继续使用：

```

Adafruit_MPU6050
```

对于 MPU6500，采用直接寄存器读取。



配置：

```

PWR_MGMT_1 = 0x01
GYRO_CONFIG = 0x00
```

陀螺仪范围：

```

±250 dps
```

灵敏度：

```

131 LSB / (°/s)
```

读取：

```

GYRO_ZOUT_H
GYRO_ZOUT_L
```

转换：

```

raw
 ↓
degree/s
 ↓
rad/s
```

然后继续复用已有：

```

Bias 校准
Heading 积分
PID 航向校正
旋转控制
IMU Fault 安全机制
```

------

# 5. GitHub 修改

仓库：

```

Justin-fanfan/longpet-motion-mcu
```

分支：

```

add-mpu6500-detection
```

PR：

```

#1 Support MPU6500-compatible IMU detection and gyro reads
```

主要修改：

```

xiao_che/running.h
xiao_che/running.cpp
```

------

# 6. MPU6500 实机结果

最终启动日志：

```

[STOP] POWER_ON latched=0

[BOOT] SERVO_ATTACH_RESULT=0 pin=2

[IMU] detected MPU6500

[IMU] READY type=MPU6500 bias=-0.018949

[MODE] SAFE

[BOOT] READY protocol=V2 fault_reset=1
```

结论：

| 项目结果        |        |
| --------------- | ------ |
| I2C 通信        | PASS   |
| WHO_AM_I 识别   | PASS   |
| MPU6500 识别    | PASS   |
| Z 轴陀螺仪读取  | PASS   |
| Bias 标定       | PASS   |
| IMU_INIT_FAILED | 已解决 |

本次实测：

```

bias = -0.018949 rad/s
```

------

# 7. UART 接收问题

IMU 修复后发现：

```

MCU -> PC
```

方向可以正常看到：

```

[BOOT]
[IMU]
[MODE]
```

等日志。



但是发送：

```

STATUS
MODE MANUAL
MOVE FORWARD 100
```

MCU 没有任何回复。



因此判断：

```

TX 链路正常
RX 链路存在问题
```

------

# 8. ESP32 临时串口转接

由于现场没有 USB-TTL，因此使用另一块 ESP32 作为 UART 转接器。



最初代码使用：

```

readBytesUntil()
```

后来为了避免：



-  字符串缓冲； 
-  read timeout； 
-  CR/LF； 
-  双重封装； 



直接改成透明字节透传：

```

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 6, 7);
}

void loop() {
    while (Serial.available()) {
        Serial1.write(Serial.read());
    }

    while (Serial1.available()) {
        Serial.write(Serial1.read());
    }
}
```

最终 UART 双向通信成功。



当前运动 MCU：

```

Serial1
115200 8N1
RX GPIO6
TX GPIO7
```

------

# 9. 底盘实机测试

进入：

```

MODE MANUAL
```

后进行了主要方向测试。



结果：

| 动作状态 |          |
| -------- | -------- |
| 前进     | 正常     |
| 后退     | 正常     |
| 左转     | 正常     |
| 右转     | 正常     |
| 左平移   | 存在问题 |
| 右平移   | 存在问题 |

对应指令：

```

MOVE FORWARD
MOVE BACKWARD
MOVE ROTATE_LEFT
MOVE ROTATE_RIGHT
MOVE SHIFT_LEFT
MOVE SHIFT_RIGHT
```

由于当前 LongPet 功能暂时不依赖左右平移，因此：

```

SHIFT_LEFT
SHIFT_RIGHT
```

问题暂缓处理。



后续若处理，应重点检查：



-  麦克纳姆轮安装方向； 
-  四个电机的正负方向； 
-  A/B/C/D 电机映射； 
-  平移时四轮符号组合； 
-  编码器方向； 
-  PID 输出符号。 

------

# 10. 手动运动安全租约

当前：

```

kManualCommandTimeoutMs = 500
```

因此单次：

```

MOVE FORWARD 30
```

只允许车辆运动约 500ms。



持续运动需要上层：

```

每 100~200ms 重发 MOVE
```

例如：

```

MOVE FORWARD 30
MOVE FORWARD 30
MOVE FORWARD 30
...
```

超时后：

```

[STOP] MANUAL_COMMAND_TIMEOUT latched=0
```

这样可以防止：

```

龙芯程序崩溃
UART 断线
遥控程序失去连接
```

以后车辆继续失控运行。



`PING` 不会刷新该运动租约。

------

# 11. 舵机实机测试

舵机：

```

GPIO2
```

参数：

```

Center = 1570 us
Min = 870 us
Max = 2270 us
```

使用：

```

HEAD CENTER
HEAD LEFT 100
HEAD RIGHT 100
```

进行了测试。



结果：

```

左右转动正常
回中正常
```

实际机械总行程约：

```

120°
```

因此：

```

Servo PWM        PASS
Servo Control    PASS
Servo Mechanics  PASS
```

暂时没有必要进一步扩大 PWM 范围，以避免舵机碰撞机械限位。

------

# 12. 底盘 + 舵机并发控制

当前 MANUAL 模式设计允许：

```

MOVE
+
HEAD
```

同时工作。



测试程序采用：

```

每约 150ms 发送 MOVE FORWARD
```

同时插入：

```

HEAD LEFT
HEAD RIGHT
HEAD CENTER
```

用于验证：



-  运动 watchdog； 
-  底盘与舵机互不覆盖； 
-  UART 命令可以交错发送； 
-  后续人工遥控可以同时控制车体与头部。 

------

# 13. 当前模式定义

## SAFE

```

禁止底盘运动
```

## HEAD_ONLY

```

允许视觉控制头部
禁止视觉驱动底盘
```

## MANUAL

```

允许手动底盘
允许手动舵机
```

## FOLLOW

```

接受视觉目标
保留后续自动跟随接口
当前自动底盘跟随关闭
```

------

# 14. 视觉接口

后续龙芯视觉模块发送：

```

TARGET <dx> <dy> <area>
```

其中：

```

dx
```

表示人物中心与画面中心的水平偏差。

```

dy
```

表示垂直偏差。

```

area
```

表示人物 bbox：

```

width × height
```

目标丢失：

```

TARGET 0 0 0
```

------

# 15. 当前视觉接入建议

当前建议首先完成：

```

Tinyissimo Detector
        +
Sparse LK Tracker
        ↓
latest_target
        ↓
独立 10Hz UART Publisher
        ↓
TARGET dx dy area
        ↓
MODE HEAD_ONLY
        ↓
头部跟踪人物
```

暂时不启用自动底盘 FOLLOW。



UART Publisher 应与 Detector 解耦。



原因是检测器推理可能需要：

```

约 0.7~1.5 秒
```

而 Sparse LK tracker 可以提供更高频率的位置更新。



因此 Publisher 应固定约：

```

10 Hz
```

持续发送最新有效目标。

------

# 16. 本轮最终硬件状态

| 模块状态              |              |
| --------------------- | ------------ |
| ESP32-S3 启动         | 正常         |
| SAFE 上电停车         | 正常         |
| UART TX               | 正常         |
| UART RX               | 正常         |
| 双向 UART             | 正常         |
| MPU6500 检测          | 正常         |
| MPU6500 Gyro Z        | 正常         |
| Bias 校准             | 正常         |
| 前进                  | 正常         |
| 后退                  | 正常         |
| 左转                  | 正常         |
| 右转                  | 正常         |
| 左平移                | 有问题，暂缓 |
| 右平移                | 有问题，暂缓 |
| 舵机左转              | 正常         |
| 舵机右转              | 正常         |
| 舵机回中              | 正常         |
| 舵机总机械范围        | 约 120°      |
| STOP                  | 已实现       |
| MANUAL 500ms Watchdog | 已实现       |
| HEAD_ONLY 底盘保护    | 已实现       |
| FOLLOW 自动底盘       | 当前关闭     |

------

# 17. 后续建议

下一阶段可以正式进入龙芯主控接入。



建议顺序：

```

1. 龙芯 UART 与 ESP32-S3 双向打通
2. STATUS / PING / STOP
3. MANUAL MOVE
4. HEAD 控制
5. HEAD_ONLY
6. 视觉 TARGET
7. Detector + Tracker 10Hz Publisher
8. 头部人体跟踪
9. 最后再考虑自动 FOLLOW
```

仍需补测的安全项目：

```

STOP 是否立即停车
500ms MOVE timeout
UART 物理断开停车
运动中切 MODE
PING 不延长 MOVE
IMU runtime fault
control overrun
```

左右平移暂时不是当前开发阻塞项。

------

# 18. 本轮结论

本轮解决了两个关键阻塞：

## IMU

原先：

```

Adafruit MPU6050 init failed
```

最终确认：

```

I2C address = 0x68
WHO_AM_I    = 0x70
```

即实际需要 MPU6500 兼容路径。



增加芯片识别和直接寄存器 Gyro 读取后：

```

[IMU] detected MPU6500
[IMU] READY
```

实机验证成功。

## UART

原先：

```

只能收到 MCU 输出
无法向 MCU 发送命令
```

使用另一块 ESP32 做透明 UART 转接后成功打通：

```

PC <-> 转接 ESP32 <-> LongPet ESP32-S3
```

目前运动 MCU 已经具备进入 LongPet 龙芯端和视觉模块正式联调阶段的条件。

