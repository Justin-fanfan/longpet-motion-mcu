# LongPet ESP32-S3 Motion Controller

LongPet 的 ESP32-S3 运动执行固件，负责头部 Servo、四轮麦克纳姆底盘、编码器 PID、
MPU6050 / MPU6500-compatible 航向反馈、UART 命令执行和安全 watchdog。

本仓库只包含运动 MCU 固件与相关台架/协议文档，不包含 LongPet 主仓库、家属端 UI、
网络接口、视觉模型或龙芯侧应用代码。

## 工程入口

- Arduino Sketch：`xiao_che/xiao_che.ino`
- 运动控制：`xiao_che/running.h`、`xiao_che/running.cpp`
- 安全与控制参数：`xiao_che/motion_config.h`
- 串口与视觉接口教程：[`docs/LongPet_UART协议与视觉接口使用说明.md`](docs/LongPet_UART协议与视觉接口使用说明.md)
- 本轮实机联调报告：[`docs/LongPet_运动MCU联调工作报告_2026-09-11.md`](docs/LongPet_运动MCU联调工作报告_2026-09-11.md)
- Motion Protocol V2：[`docs/motion-protocol-v2.md`](docs/motion-protocol-v2.md)
- 固件基线：[`docs/firmware-baseline.md`](docs/firmware-baseline.md)
- 台架验收记录：[`docs/bench-test-report.md`](docs/bench-test-report.md)

## 当前硬件与串口

运动 MCU：ESP32-S3。

控制和诊断共用 `Serial1`：

```text
115200 baud
8 data bits
no parity
1 stop bit
no flow control
RX = GPIO6
TX = GPIO7
```

控制端必须交叉连接：

```text
控制端 TX  -> ESP32-S3 GPIO6 RX
控制端 RX  <- ESP32-S3 GPIO7 TX
控制端 GND -> ESP32-S3 GND
```

没有 USB-TTL 时，可以使用另一块 ESP32 做透明 UART 桥；本轮已经通过该方式验证双向收发。

## 当前控制模式

- `SAFE`：默认安全模式，底盘禁止运动。
- `HEAD_ONLY`：接受 Vision `TARGET`，只驱动头部，底盘硬禁止运动。
- `MANUAL`：允许独立控制头部和底盘；底盘 `MOVE` 必须持续刷新。
- `FOLLOW`：接收 `TARGET` 并可驱动头部；自动底盘跟随当前仍关闭，等待 bbox 距离阈值实机标定。

只有**实际切换到不同模式**时才会执行模式切换停车与瞬态状态清理；重复发送当前模式是 no-op。

## UART Motion Protocol V2

所有命令均为 ASCII 单行文本，推荐使用 `CRLF` 结尾。

```text
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

完整命令、合法模式、参数范围和 watchdog 语义见
[`docs/motion-protocol-v2.md`](docs/motion-protocol-v2.md)。协议不使用 JSON 或二进制 framing。

`STATUS`、启动日志、故障日志和命令诊断都从同一条 `Serial1` 返回，因此龙芯侧只需要维护一条双向 UART。

## MANUAL 运动租约

`MOVE` 不是无限保持命令。当前：

```text
link timeout          = 500 ms
manual MOVE lease     = 500 ms
target freshness      = 500 ms
```

持续运动时建议每 `100~200 ms` 重发当前 `MOVE`。

当前代码在活动底盘上先检查 general link timeout，再检查 MANUAL lease，因此：

- 如果发送一次 `MOVE` 后完全没有任何合法串口命令，约 500 ms 后通常记录 `LINK_TIMEOUT`；
- 如果持续用 `PING` / `STATUS` 等保持 general link 新鲜、但不刷新 `MOVE`，约 500 ms 后记录 `MANUAL_COMMAND_TIMEOUT`；
- 两种情况都会安全停车。

`PING` 不能延长 MANUAL `MOVE` lease，也不能延长视觉目标 TTL。

## 视觉模块数据接口

视觉模块只需发送：

```text
TARGET <dx> <dy> <area>
```

推荐定义：

```text
dx   = target_center_x - image_center_x
dy   = target_center_y - image_center_y
area = bbox_width * bbox_height
```

- `dx < 0`：目标在画面左侧；`dx > 0`：目标在画面右侧。
- `dy` 当前只校验和保存，留给后续垂直方向控制。
- `area` 当前作为 bbox 像素面积，不是物理距离。
- `area == 0` 表示目标丢失，推荐发送 `TARGET 0 0 0`。

当前视觉联调推荐：

```text
MODE HEAD_ONLY
TARGET dx dy area
TARGET dx dy area
...
```

Detector + Tracker 架构应由独立 UART Publisher 以约 `10 Hz` 发布最新且仍然新鲜的目标，
不要把发送节奏绑定到较慢的 Detector 推理完成事件。

## IMU 兼容

当前固件支持：

```text
WHO_AM_I = 0x68 -> MPU6050
WHO_AM_I = 0x70 -> MPU6500-compatible
I2C address      -> 0x68
```

MPU6050 继续使用 `Adafruit_MPU6050`；实机 MPU6500-compatible 走直接寄存器路径读取 Gyro Z，
之后统一进入 bias 校准、heading 积分和 PID 航向控制。

本轮实机已经成功得到：

```text
[IMU] detected MPU6500
[IMU] READY type=MPU6500 bias=-0.018949
```

原 `IMU_INIT_FAILED` 已解决。

## 舵机

```text
GPIO       = 2
center     = 1570 us
software   = 870 ~ 2270 us
max step   = 100 us / command
```

实机已经确认 `HEAD LEFT`、`HEAD RIGHT`、`HEAD CENTER` 正常，机械总活动范围约 `120°`。

## 重要安全边界

- 上电首先进入停车状态并保持 `SAFE`。
- `POWER_ON`、`STOP_COMMAND`、`TARGET_LOST`、`LINK_TIMEOUT`、`MODE_CHANGED`、`MANUAL_COMMAND_TIMEOUT` 为可恢复停车。
- IMU 运行故障、转弯超时、控制周期严重超期为锁存故障，必须复位后恢复。
- `HEAD_ONLY` 在执行器分发层硬阻止轮子动作。
- `FOLLOW` 当前不执行自动底盘追踪。
- `MOVE SHIFT_LEFT/RIGHT` 协议保留，但本轮实机表现存在问题，现阶段不要作为依赖功能。
- 软件不能替代硬件急停；正式落地仍建议 STBY 外部下拉和独立硬件急停。

当前实现还有一个诊断限制：锁存故障后若再执行 `STOP` 或实际模式切换，`faultLatched` 仍保持为 1，
但单一 `stopReason` 字段可能被新的停车原因覆盖。因此排故时以首次 `[FAULT] ...` 日志为根因依据。

## 当前实机验证状态

截至 2026-09-11：

| 项目 | 状态 |
|---|---|
| ESP32-S3 启动 / SAFE | ✅ 正常 |
| UART TX / RX | ✅ 双向已打通 |
| MPU6500-compatible 检测 | ✅ 正常 |
| Gyro Z / bias 校准 | ✅ 正常 |
| `MOVE FORWARD` | ✅ 正常 |
| `MOVE BACKWARD` | ✅ 正常 |
| `MOVE ROTATE_LEFT` | ✅ 正常 |
| `MOVE ROTATE_RIGHT` | ✅ 正常 |
| `MOVE SHIFT_LEFT` | ⚠️ 实机存在问题，暂缓 |
| `MOVE SHIFT_RIGHT` | ⚠️ 实机存在问题，暂缓 |
| Servo LEFT / RIGHT / CENTER | ✅ 正常 |
| Servo 实际总行程 | ✅ 约 120° |
| FOLLOW 自动底盘 | ⏸️ 当前故意关闭 |
| STOP / timeout / 断线 / fault 全套安全矩阵 | ⏳ 仍需逐项补测 |
| 底盘 + Servo 并发 | ⏳ 已准备测试方法，尚未记录最终实测结论 |

## 后续联调顺序

1. 龙芯 UART 与 ESP32-S3 双向通信。
2. `PING` / `STATUS` / `STOP`。
3. `MODE MANUAL` + 已验证的四个基础运动方向。
4. `HEAD` 控制。
5. `MODE HEAD_ONLY`。
6. 视觉端发布 `TARGET dx dy area`。
7. Detector + Tracker 独立约 10 Hz UART Publisher。
8. 完成头部人物跟踪。
9. 最后再进入 FOLLOW 距离标定与底盘策略。

详细待测项见 [`docs/bench-test-report.md`](docs/bench-test-report.md)。

## 工具链基线

```text
Board: ESP32S3 Dev Module
Arduino ESP32 Core 2.0.14
xtensa-esp32s3-elf-gcc esp-2021r2-patch5-8.4.0
Adafruit MPU6050 2.2.9
Adafruit BusIO 1.17.4
Adafruit Unified Sensor 1.1.15
ESP32Encoder 0.12.0
PID 1.2.0
Servo 1.3.0
DHT 1.4.7
```

当前固件已经完成实际编译、烧录和板卡联调；具体 Arduino Board 菜单、Flash/PSRAM/USB 选项仍建议在后续可复现构建文档中固定记录。
