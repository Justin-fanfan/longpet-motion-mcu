# LongPet ESP32-S3 Motion Controller

LongPet 的 ESP32-S3 运动执行固件，负责头部 Servo、四轮麦克纳姆底盘、编码器 PID、
MPU6050 / MPU6500-compatible 航向反馈、UART 命令执行和安全 watchdog。

本仓库只包含运动 MCU 固件与相关台架/协议文档，不包含 LongPet 主仓库、家属端 UI、
网络接口、视觉模型或龙芯侧应用代码。

## 工程入口

- Arduino Sketch：`xiao_che/xiao_che.ino`
- 运动控制：`xiao_che/running.h`、`xiao_che/running.cpp`
- 安全与控制参数：`xiao_che/motion_config.h`
- Motion Protocol V2：`docs/motion-protocol-v2.md`
- 固件基线：`docs/firmware-baseline.md`
- 台架测试：`docs/bench-test-report.md`

## 当前硬件与串口

运动 MCU：ESP32-S3。

控制/诊断 UART 使用 `Serial1`：

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

没有 USB-TTL 时，可以使用另一块 ESP32 做透明 UART 桥。当前实机已经通过该方式验证双向收发。

## 当前控制模式

- `SAFE`：默认安全模式，底盘禁止运动。
- `HEAD_ONLY`：接受 Vision `TARGET`，只驱动头部，底盘硬禁止运动。
- `MANUAL`：允许独立控制头部和底盘；底盘 `MOVE` 必须持续刷新。
- `FOLLOW`：接收 `TARGET` 并可驱动头部；自动底盘跟随当前仍关闭，等待 bbox 距离阈值实机标定。

## UART Motion Protocol V2

所有命令均为 ASCII 单行文本，推荐使用 `CRLF` 结尾。

常用命令：

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

示例：

```text
MODE MANUAL
MOVE FORWARD 30
HEAD RIGHT 100
STOP
```

完整命令、参数范围、模式合法性、legacy `dx dy area` 兼容和发送节奏见
[`docs/motion-protocol-v2.md`](docs/motion-protocol-v2.md)。协议不使用 JSON 或二进制 framing。

## MANUAL 运动租约

`MOVE` 不是无限保持命令。

当前：

```text
MANUAL command lease = 500 ms
```

如果上层希望持续运动，应每约 `100~200 ms` 重发当前 `MOVE`：

```text
MOVE FORWARD 30
MOVE FORWARD 30
MOVE FORWARD 30
...
```

超过约 500 ms 没有新的运动命令时，MCU 自动停车并报告：

```text
[STOP] MANUAL_COMMAND_TIMEOUT latched=0
```

`PING` 只维持通信链路，不会刷新 MANUAL 运动租约，也不会刷新视觉目标 TTL。

## 视觉模块数据接口

视觉模块不需要向运动 MCU 发送图像或完整检测框，只发送：

```text
TARGET <dx> <dy> <area>
```

推荐定义：

```text
dx   = target_center_x - image_center_x
dy   = target_center_y - image_center_y
area = bbox_width * bbox_height
```

其中：

- `dx < 0`：目标在画面左侧；`dx > 0`：目标在画面右侧。
- `dy` 保留给后续垂直方向控制或更完整的视觉逻辑。
- `area` 用作人物距离的粗略代理量。
- `area == 0` 表示目标丢失，建议发送 `TARGET 0 0 0`。

当前推荐视觉联调模式：

```text
MODE HEAD_ONLY
TARGET dx dy area
TARGET dx dy area
...
```

这样视觉可以驱动头部跟踪，但绝不会驱动底盘。

对于 Detector + Tracker 架构，建议由独立 UART Publisher 以约 `10 Hz` 发布最新且仍然新鲜的目标，
不要把 UART 发送绑定到较慢的 Detector 推理完成事件。

## IMU 兼容

当前固件支持两条 IMU 路径：

```text
WHO_AM_I = 0x68 -> MPU6050
WHO_AM_I = 0x70 -> MPU6500-compatible
```

I2C 总线地址仍为：

```text
0x68
```

对于 MPU6050，继续使用 `Adafruit_MPU6050`。

对于实机检测到的 MPU6500-compatible 器件，固件使用直接寄存器方式配置并读取 Gyro Z，
然后统一进入现有 bias 校准、heading 积分、PID 航向修正和旋转控制逻辑。

实机已经成功得到：

```text
[IMU] detected MPU6500
[IMU] READY type=MPU6500 bias=...
```

原先的 `IMU_INIT_FAILED` 已解决。

## 舵机

当前头部舵机：

```text
GPIO       = 2
center     = 1570 us
software   = 870 ~ 2270 us
max step   = 100 us / command
```

控制命令：

```text
HEAD LEFT 100
HEAD RIGHT 100
HEAD CENTER
```

实机已经确认左右转动和回中正常，当前机械总活动范围约 `120°`。

## 重要安全边界

- 上电首先进入停车状态并保持 `SAFE`。
- TB6612 的 `STBY_AB` / `STBY_CD` 用于禁止电机驱动；软件安全不能替代硬件急停。
- `POWER_ON`、`STOP_COMMAND`、`TARGET_LOST`、`LINK_TIMEOUT`、`MODE_CHANGED` 和 `MANUAL_COMMAND_TIMEOUT` 属于可恢复停车。
- IMU 运行故障、转弯超时、控制周期严重超期属于锁存故障，必须复位后恢复。
- `MOVE` 使用约 500 ms lease；不能依赖单次命令永久运动。
- `HEAD_ONLY` 模式在执行器分发层硬阻止轮子动作。
- `FOLLOW` 当前不执行自动底盘追踪，避免未标定的 bbox 面积阈值直接驱动车辆。
- 第一次测试任何新的底盘动作时仍应架空四轮。
- 软件不能覆盖 MCU 卡死、异常供电或复位期间 GPIO 高阻窗口；正式落地建议保留 STBY 外部下拉和独立硬件急停。

## 当前实机验证状态

截至 2026-09-11：

| 项目 | 状态 |
|---|---|
| ESP32-S3 启动 / SAFE 停车 | ✅ 正常 |
| UART TX | ✅ 正常 |
| UART RX | ✅ 正常 |
| 双向 UART 命令链路 | ✅ 已打通 |
| MPU6500-compatible 检测 | ✅ 正常 |
| Gyro Z 读取 | ✅ 正常 |
| IMU bias 校准 | ✅ 正常 |
| `MOVE FORWARD` | ✅ 正常 |
| `MOVE BACKWARD` | ✅ 正常 |
| `MOVE ROTATE_LEFT` | ✅ 正常 |
| `MOVE ROTATE_RIGHT` | ✅ 正常 |
| `MOVE SHIFT_LEFT` | ⚠️ 实机存在问题，暂缓 |
| `MOVE SHIFT_RIGHT` | ⚠️ 实机存在问题，暂缓 |
| Servo LEFT / RIGHT / CENTER | ✅ 正常 |
| Servo 实际总行程 | ✅ 约 120° |
| MPU6500 初始化故障 | ✅ 已修复 |
| FOLLOW 自动底盘 | ⏸️ 当前故意关闭 |

左右平移不是当前 LongPet 版本的阻塞项，后续如需要再检查麦克纳姆轮安装方向、四电机符号组合、
编码器方向与平移 PID 输出。

## 后续联调顺序

当前 MCU 已具备进入龙芯主控和视觉模块正式联调的条件。建议按以下顺序继续：

1. 龙芯 UART 与 ESP32-S3 双向通信。
2. `PING` / `STATUS` / `STOP`。
3. `MODE MANUAL` + 四个当前已验证的基础运动方向。
4. `HEAD` 控制。
5. `MODE HEAD_ONLY`。
6. 视觉端发布 `TARGET dx dy area`。
7. Detector + Tracker 独立约 10 Hz UART Publisher。
8. 完成头部人物跟踪后，再进入自动 FOLLOW 设计与距离阈值标定。

仍建议补完的安全实测包括：运动中 `STOP`、500 ms lease、UART 物理断线、运动中模式切换、
`PING` 不延长 MOVE、IMU runtime fault 和 control overrun。

## 工具链基线

当前已知开发环境：

```text
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

当前固件已经完成实际编译、烧录和板卡联调；具体板卡菜单、Flash/PSRAM/USB 选项仍建议在后续可复现构建文档中固定记录。
