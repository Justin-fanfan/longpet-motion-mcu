# LongPet 运动 MCU 联调与 MPU6500 兼容工作报告

> 项目：LongPet  
> 运动 MCU：ESP32-S3  
> 日期：2026-09-11

---

## 1. 本轮目标

本轮完成的核心工作：

1. 排查并修复 IMU 初始化失败；
2. 确认实际芯片为 MPU6500-compatible；
3. 增加 MPU6050 / MPU6500 双路径；
4. 实机验证 MPU6500 Gyro Z 与 bias；
5. 排查“能看到 MCU 输出、但 MCU 收不到命令”的 UART 问题；
6. 使用另一块 ESP32 临时作为双向串口桥；
7. 打通 V2 双向 UART；
8. 验证前进、后退、原地左右转；
9. 发现左右平移问题并决定暂缓；
10. 验证头部舵机左右与回中；
11. 整理后续龙芯主控与视觉接口。

---

## 2. 初始 IMU 故障

初始启动表现：

```text
[STOP] POWER_ON latched=0
[BOOT] SERVO_ATTACH_RESULT=0 pin=2
[STOP] IMU_INIT_FAILED latched=1
```

已知现象：

- I2C 扫描可以看到设备；
- `Adafruit_MPU6050::begin()` 失败；
- 固件锁存 `IMU_INIT_FAILED`；
- 底盘运动被安全逻辑阻止。

---

## 3. IMU 芯片确认

原始寄存器测试：

```text
WHO_AM_I 0x75 = 0x70
PWR_MGMT_1 0x6B = 0x01
```

I2C 总线地址：

```text
0x68
```

因此需要明确区分：

```text
0x68 = I2C address
0x70 = WHO_AM_I chip identity
```

当前板上器件应按 MPU6500-compatible 处理，而不是把 `0x70` 错当成新的 I2C 地址。

原 `Adafruit_MPU6050` 路径会检查设备身份，因此总线通信正常并不代表该库一定接受芯片。

---

## 4. MPU6500 兼容实现

新增 IMU 类型：

```cpp
enum class ImuType : uint8_t {
    Unknown,
    MPU6050,
    MPU6500
};
```

初始化读取：

```text
WHO_AM_I register = 0x75
```

分流：

```text
0x68 -> MPU6050
0x70 -> MPU6500-compatible
other -> unsupported
```

### MPU6050 路径

继续使用：

```text
Adafruit_MPU6050
```

### MPU6500-compatible 路径

直接寄存器配置：

```text
PWR_MGMT_1 = 0x01
GYRO_CONFIG = 0x00
```

Gyro 范围：

```text
±250 dps
```

灵敏度：

```text
131 LSB / (°/s)
```

读取：

```text
GYRO_ZOUT_H
GYRO_ZOUT_L
```

转换：

```text
raw -> degree/s -> rad/s
```

之后继续复用已有：

- bias 校准；
- heading 积分；
- heading PID；
- 前进/后退/平移航向保持；
- 左右旋转；
- IMU runtime fault 安全路径。

---

## 5. GitHub 修改状态

仓库：

```text
Justin-fanfan/longpet-motion-mcu
```

MPU6500 修改通过 PR：

```text
#1 Support MPU6500-compatible IMU detection and gyro reads
```

该 PR 已合并到 `main`。

主要代码：

```text
xiao_che/running.h
xiao_che/running.cpp
```

UART V2 主入口：

```text
xiao_che/xiao_che.ino
```

---

## 6. MPU6500 实机结果

最终启动日志：

```text
[STOP] POWER_ON latched=0
[BOOT] SERVO_ATTACH_RESULT=0 pin=2
[IMU] detected MPU6500
[IMU] READY type=MPU6500 bias=-0.018949
[MODE] SAFE
[BOOT] READY protocol=V2 fault_reset=1
```

结果：

| 项目 | 结论 |
|---|---|
| I2C 通信 | PASS |
| WHO_AM_I 读取 | PASS |
| MPU6500-compatible 识别 | PASS |
| Gyro Z | PASS |
| bias 标定 | PASS |
| `IMU_INIT_FAILED` | 已解决 |

本次 bias：

```text
-0.018949 rad/s
```

---

## 7. UART 单向问题

IMU 修复后最初出现：

```text
Motion MCU -> PC/bridge     正常
PC/bridge -> Motion MCU     无响应
```

能够看到：

```text
[BOOT]
[IMU]
[MODE]
```

但发送：

```text
STATUS
MODE MANUAL
MOVE FORWARD 100
```

没有回复。

这说明“能看到启动日志”只能证明 MCU TX 方向正常，不能证明 MCU RX 方向已经打通。

---

## 8. 使用另一块 ESP32 做透明 UART 桥

现场没有独立 USB-TTL，因此使用第二块 ESP32。

原桥接代码使用 `readBytesUntil()` 做字符串转发。为了排除 read timeout、CR/LF 和二次缓冲的影响，改成纯字节透传：

```cpp
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

最终双向通信打通。

Motion MCU 当前配置：

```text
Serial1
115200 8N1
RX = GPIO6
TX = GPIO7
```

接线原则：

```text
bridge/host TX -> LongPet GPIO6 RX
bridge/host RX <- LongPet GPIO7 TX
GND            -> GND
```

---

## 9. 底盘实机测试

进入：

```text
MODE MANUAL
```

后测试主要动作。

| 动作 | 结果 |
|---|---|
| `MOVE FORWARD` | 正常 |
| `MOVE BACKWARD` | 正常 |
| `MOVE ROTATE_LEFT` | 正常 |
| `MOVE ROTATE_RIGHT` | 正常 |
| `MOVE SHIFT_LEFT` | 存在问题 |
| `MOVE SHIFT_RIGHT` | 存在问题 |

由于当前 LongPet 版本暂时不依赖横向平移，决定先继续主链路开发，不在本轮阻塞于 SHIFT。

后续排查 SHIFT 时优先检查：

- 麦克纳姆轮安装方向；
- A/B/C/D 电机物理位置；
- 四轮正反号；
- 编码器方向；
- shift 轮速组合；
- PID 输出方向。

---

## 10. MANUAL watchdog 的准确语义

当前参数：

```text
kLinkTimeoutMs          = 500
kManualCommandTimeoutMs = 500
```

持续运动时建议：

```text
每 100~200 ms 重发 MOVE
```

需要注意当前代码的检查顺序：

1. general link timeout；
2. MANUAL motion lease timeout。

所以：

- 一条 `MOVE` 后完全不再发送任何合法命令，约 500 ms 后通常记录 `LINK_TIMEOUT`；
- 停止 `MOVE`，但继续发 `PING` / `STATUS` 维持 general link，约 500 ms 后记录 `MANUAL_COMMAND_TIMEOUT`。

两种情况都会停车。

本轮还没有逐项记录完整 watchdog / 断线验收日志，因此这些安全项保留为后续测试，而不是标记为“全部通过”。

---

## 11. 舵机实机测试

Servo：

```text
GPIO2
center = 1570 us
software range = 870..2270 us
```

测试：

```text
HEAD LEFT 100
HEAD RIGHT 100
HEAD CENTER
```

结果：

```text
左右转动正常
回中正常
机械总活动范围约 120°
```

因此本轮可确认：

```text
Servo signal/control      PASS
LEFT/RIGHT/CENTER         PASS
mechanical movement       PASS
```

启动日志中的：

```text
[BOOT] SERVO_ATTACH_RESULT=0 pin=2
```

不能单独据此判断舵机失败，因为物理动作已经证明当前 Servo 链路可用。

本轮没有记录“底盘持续运行 + Servo 同时动作”的最终验收结果，因此并发项仍标记待测。

---

## 12. 当前 V2 模式

### SAFE

```text
底盘禁止运动
```

### HEAD_ONLY

```text
视觉 TARGET 可驱动头部
底盘硬阻止
```

### MANUAL

```text
允许 MOVE
允许 HEAD
```

### FOLLOW

```text
接收 TARGET
允许头部跟踪
自动底盘当前关闭
```

重复发送当前模式是 no-op；只有真正切换到不同模式才执行模式切换停车与瞬态清理。

---

## 13. 视觉接口

龙芯视觉模块后续发送：

```text
TARGET <dx> <dy> <area>
```

定义：

```text
dx   = target_center_x - image_center_x
dy   = target_center_y - image_center_y
area = bbox_width * bbox_height
```

目标丢失：

```text
TARGET 0 0 0
```

当前 `area` 是像素面积，不是实际距离。

5000/10000 旧阈值仍保留在配置中，但当前 FOLLOW 不用它们驱动底盘。

---

## 14. 当前视觉接入建议

当前视觉链路：

```text
Tinyissimo Detector
        +
Sparse LK Tracker
        ↓
latest_target
        ↓
独立固定频率 UART Publisher
        ↓
TARGET dx dy area
        ↓
MODE HEAD_ONLY
        ↓
头部跟踪
```

推荐 Publisher：

```text
约 10 Hz
```

原因：Detector correction 较慢时，不能让 UART 在约 0.7~1.5 s 的推理期间完全停止；
应持续发布 Tracker 提供的最新且仍然新鲜的目标。

推荐视觉侧 stale 门槛约 300 ms；过期后发送：

```text
TARGET 0 0 0
```

MCU 自身 target timeout 为 500 ms。

---

## 15. 当前已确认状态

| 模块 | 状态 |
|---|---|
| ESP32-S3 启动 / SAFE | 正常 |
| UART TX | 正常 |
| UART RX | 正常 |
| 双向 V2 UART | 正常 |
| MPU6500-compatible 检测 | 正常 |
| Gyro Z | 正常 |
| bias | 正常 |
| 前进 | 正常 |
| 后退 | 正常 |
| 原地左转 | 正常 |
| 原地右转 | 正常 |
| 左平移 | 有问题，暂缓 |
| 右平移 | 有问题，暂缓 |
| Servo 左/右/中 | 正常 |
| Servo 总机械范围 | 约 120° |
| FOLLOW 自动底盘 | 当前关闭 |
| 完整 STOP/watchdog/断线/fault 安全矩阵 | 尚未全部验收 |
| Servo + chassis 并发 | 尚未记录最终结论 |
| HEAD_ONLY 视觉闭环 | 待龙芯视觉端接入 |

---

## 16. 当前已知诊断限制

严重故障发生时会设置：

```text
faultLatched = 1
```

但当前只有一个 `stopReason`。fault 后如果再发送 `STOP` 或真正切换模式，
`faultLatched` 不会被清除，但 `stopReason` 可能被新的原因覆盖。

因此严重故障排查必须优先保留首次：

```text
[FAULT] ...
```

日志。

---

## 17. 下一阶段

推荐顺序：

1. 龙芯 UART 与 ESP32-S3 双向打通；
2. `STATUS` / `PING` / `STOP`；
3. MANUAL 基础动作；
4. HEAD；
5. HEAD_ONLY；
6. 视觉 `TARGET`；
7. Detector + Tracker 约 10 Hz Publisher；
8. 头部人物跟踪；
9. 补齐 STOP / timeout / 断线 / fault 安全矩阵；
10. 最后再进入 FOLLOW 距离标定与自动底盘策略。

---

## 18. 本轮结论

本轮解决了两个主要阻塞：

### IMU

```text
I2C address = 0x68
WHO_AM_I    = 0x70
```

通过新增 MPU6500-compatible 直接寄存器 Gyro 路径，最终得到：

```text
[IMU] detected MPU6500
[IMU] READY type=MPU6500 bias=-0.018949
```

### UART

从“只能看到 MCU 输出”推进到：

```text
PC <-> bridge ESP32 <-> LongPet ESP32-S3
```

双向命令链路正常。

在此基础上，前进、后退、原地左右转和头部舵机已经通过实机基础验证；
左右平移已知有问题但暂不阻塞。当前 MCU 已具备进入龙芯主控和视觉 HEAD_ONLY 正式联调阶段的条件。
