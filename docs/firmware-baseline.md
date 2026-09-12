# LongPet ESP32-S3 Motion Controller 固件基线

日期：2026-09-12

本文档记录当前 `main` 分支的 V2 固件行为、硬件资源、已验证结论与仍未验证项。
历史 V1 自动跟随逻辑已经退出当前执行路径；协议细节以
[`motion-protocol-v2.md`](motion-protocol-v2.md) 为准，实机过程见
[`LongPet_运动MCU联调工作报告_2026-09-11.md`](LongPet_运动MCU联调工作报告_2026-09-11.md)。

---

## 1. 当前控制结构

```text
Serial1 bounded byte poll
  -> strict V2 parser / legacy target parser
  -> mode legality + freshness timestamps
  -> recoverable stop / latched fault
  -> MANUAL MOVE / FOLLOW_MOVE ownership dispatch
  -> Run::Forward/Backward/LeftShift/RightShift/RotateLeft/RotateRight
  -> encoder PID + heading PID
  -> LEDC 4..7 + TB6612 STBY
```

控制模式：

| 模式 | 当前行为 |
|---|---|
| `SAFE` | 默认模式；底盘保持停止 |
| `HEAD_ONLY` | `TARGET` 只驱动头部；底盘执行路径硬阻断 |
| `MANUAL` | `HEAD` 与新鲜 `MOVE` 独立工作 |
| `FOLLOW` | `TARGET` 跟头；只执行独立租约的 FORWARD / ROTATE_LEFT / ROTATE_RIGHT / STOP |

只有真正切换到不同模式时才执行模式切换停车与瞬态状态清理；重复发送当前模式是 no-op。

---

## 2. UART 基线

控制和诊断共用 `Serial1`：

```text
baud     = 115200
format   = 8N1
flow     = none
RX       = GPIO6
TX       = GPIO7
```

当前所有这些输出都走 `Serial1`：

- boot log；
- IMU log；
- `[MODE]` / `[MOTION]` / `[STOP]`；
- `[FAULT]`；
- `[LINK]`；
- `[STATUS]`；
- DHT `[ENV]`。

本轮使用另一块 ESP32 做透明 UART 桥，已经验证双向收发可用。

---

## 3. Parser 与协议边界

当前：

```text
line buffer            = 64 bytes
max bytes per loop     = 32
CR / LF / CRLF         = accepted
```

非法命令、越界整数、额外字段、控制字符或超长行不会刷新任何 timeout。

旧三整数格式：

```text
<dx> <dy> <area>
```

仅作为 `TARGET <dx> <dy> <area>` 的兼容写法，必须在 `HEAD_ONLY` / `FOLLOW` 使用，
不会恢复旧版自动前进或舵机极限自动转弯。

---

## 4. Stop 与 Fault

### 可恢复停车

- `POWER_ON`
- `STOP_COMMAND`
- `TARGET_LOST`
- `LINK_TIMEOUT`
- `MODE_CHANGED`
- `MANUAL_COMMAND_TIMEOUT`
- `TARGET_TRACKING_ONLY`
- `FOLLOW_CHASSIS_DISABLED`
- `FOLLOW_COMMAND_TIMEOUT`

### 锁存故障

- `IMU_INIT_FAILED`
- `IMU_RUNTIME_FAILED`
- `CONTROL_OVERRUN`
- `TURN_TIMEOUT`

所有停车最终调用 `Run::Stop()`，撤销两组 STBY、清实际电机 PWM、方向脚、编码器窗口和 PID 瞬态状态。
Servo 不因 STOP 自动回中。

### 当前诊断限制

`faultLatched` 与 `stopReason` 不是同一个概念。严重故障锁存后，如果再发送 `STOP` 或真正切换模式，
`faultLatched` 仍保持为 1，但 `stopReason` 可能被新原因覆盖。因此根因分析应优先保留首次 `[FAULT]` 日志。

---

## 5. Timeout 基线

当前四个主要时间参数都是 500 ms：

```text
kLinkTimeoutMs          = 500
kManualCommandTimeoutMs = 500
kFollowCommandTimeoutMs = 500
kTargetTimeoutMs        = 500
```

时间戳：

- `lastValidLinkCommandMs`：任意完整且模式合法的命令刷新；
- `lastManualMotionCommandMs`：仅 MANUAL `MOVE` 刷新；
- `lastTargetCommandMs`：仅 HEAD_ONLY/FOLLOW `TARGET` 刷新。
- `lastFollowMotionCommandMs`：仅 FOLLOW `FOLLOW_MOVE` 刷新；PING、STATUS、TARGET 均不刷新。

当前 MANUAL 活动底盘的检查顺序是 general link timeout 在前、manual lease 在后。因此：

- 一条 `MOVE` 后完全静默，约 500 ms 时通常记录 `LINK_TIMEOUT`；
- 用 `PING` / `STATUS` 等维持 general link，但不刷新 MOVE，则记录 `MANUAL_COMMAND_TIMEOUT`。

两种情况都会停车。

---

## 6. IMU 基线

I2C：

```text
SDA = GPIO35
SCL = GPIO36
address = 0x68
```

当前按 `WHO_AM_I` 分流：

```text
0x68 -> MPU6050
0x70 -> MPU6500-compatible
```

### MPU6050

继续使用 `Adafruit_MPU6050`。

### MPU6500-compatible

使用直接寄存器路径：

```text
PWR_MGMT_1 = 0x01
GYRO_CONFIG = 0x00   // ±250 dps
Gyro Z sensitivity = 131 LSB/(°/s)
```

读取 Gyro Z 后转换为 rad/s，继续复用现有 bias 校准、heading 积分和 heading PID。

本轮实机：

```text
WHO_AM_I = 0x70
[IMU] detected MPU6500
[IMU] READY type=MPU6500 bias=-0.018949
```

`IMU_INIT_FAILED` 已解决。

---

## 7. Servo 基线

```text
signal pin       = GPIO2
center           = 1570 us
software min     = 870 us
software max     = 2270 us
attach range     = 500..2500 us
default step     = 20 us
max command step = 100 us
```

当前方向定义：

```text
HEAD LEFT  -> physical left  -> pulse increases
HEAD RIGHT -> physical right -> pulse decreases
HEAD CENTER -> 1570 us
```

V2.2 根据后续实机反馈修正了此前相反的 LEFT/RIGHT 协议语义，`HEAD` 与 `TARGET` 现统一经过
`head_direction.h` 映射。机械总活动范围仍约 `120°`；新方向固件需要用户重烧后重新确认。

原先对 Servo PWM 资源冲突的担忧没有表现为“舵机完全不可用”：当前舵机已实机动作正常。
但本轮尚未记录 Servo 与四轮同时运行时的最终长时间并发结论，因此并发资源仍保留为待完整验收项。

---

## 8. 电机与引脚资源

| 功能 | GPIO | Motor | LEDC | 备注 |
|---|---:|---:|---:|---|
| AIN1 / AIN2 | 41 / 42 | A | — | direction |
| PWMA | 4 | A | 4 | 25 kHz, 10 bit |
| Encoder A | 11 / 12 | A | — | HalfQuad |
| BIN1 / BIN2 | 39 / 38 | B | — | direction |
| PWMB | 5 | B | 5 | 25 kHz, 10 bit |
| Encoder B | 13 / 14 | B | — | HalfQuad |
| CIN1 / CIN2 | 48 / 45 | C | — | direction |
| PWMC | 9 | C | 6 | 25 kHz, 10 bit |
| Encoder C | 15 / 16 | C | — | HalfQuad |
| DIN1 / DIN2 | 21 / 20 | D | — | direction |
| PWMD | 10 | D | 7 | 25 kHz, 10 bit |
| Encoder D | 17 / 18 | D | — | HalfQuad |
| STBY_AB | 40 | — | — | A/B enable |
| STBY_CD | 47 | — | — | C/D enable |
| Servo | 2 | — | library-managed | head |
| DHT22 | 8 | — | — | stopped-only sampling |
| Serial1 RX/TX | 6 / 7 | — | — | 115200 8N1 |

本轮物理动作结果：

```text
FORWARD       OK
BACKWARD      OK
ROTATE_LEFT   OK
ROTATE_RIGHT  OK
SHIFT_LEFT    problem / deferred
SHIFT_RIGHT   problem / deferred
```

左右平移后续应重点核查麦克纳姆轮安装方向、四电机正反定义、编码器符号与 shift 轮速组合。

---

## 9. Vision / TARGET 基线

```text
TARGET <dx> <dy> <area>
```

范围：

```text
dx   -4096..4096
dy   -4096..4096
area 0..16777216
```

当前：

- `dx` 驱动水平头部修正；
- `dy` 只保存；
- `area` 是 bbox 像素面积，不是物理距离；
- `area == 0` 表示 target lost；
- 5000 / 10000 旧阈值仍保留但不使用；距离分级由 LongPet 使用归一化 bbox 高度完成。

HEAD_ONLY 修正参数：

```text
deadband                   = 10 px
correction divisor         = 8
max correction per target  = 40 us
```

视觉侧推荐使用 Detector + Tracker 的 latest target，并由独立 Publisher 约 10 Hz 发送，而不是绑定到较慢 Detector 推理完成事件。

---

## 10. 当前验证结论

| 项目 | 当前结论 |
|---|---|
| 代码分析 | 已完成 |
| 编译 | 已完成实际编译 |
| 烧录 | 已完成 |
| 双向 UART | 已打通 |
| MPU6500-compatible | 已识别并正常读 Gyro Z |
| bias 校准 | 已完成，实测 `-0.018949 rad/s` |
| Forward / Backward | 实机正常 |
| Rotate Left / Right | 实机正常 |
| Shift Left / Right | 实机存在问题，暂缓 |
| Servo Left / Right / Center | 实机正常 |
| Servo 机械范围 | 约 120° |
| STOP / timeout / disconnect / fault 全矩阵 | 未全部逐项记录 |
| HEAD_ONLY 视觉目标实机闭环 | 用户已确认 V2.2 通过 |
| FOLLOW_MOVE 独立租约 host 测试 | 已通过 |
| V2.3 人物跟随硬件闭环 | 待用户测试 |
| Servo + chassis 并发 | 尚未记录最终验收结论 |

因此当前可以进入龙芯 UART 与视觉 HEAD_ONLY 联调，但不能把整个 35 项安全矩阵称为“全部硬件验收通过”。

---

## 11. 工具链基线

已知目标环境：

```text
Board: ESP32S3 Dev Module
Arduino ESP32 Core 2.0.14
Compiler: xtensa-esp32s3-elf-gcc esp-2021r2-patch5-8.4.0
Adafruit MPU6050 2.2.9
Adafruit BusIO 1.17.4
Adafruit Unified Sensor 1.1.15
ESP32Encoder 0.12.0
PID 1.2.0
Servo 1.3.0
DHT 1.4.7
Wire 2.0.0
SPI 2.0.0
```

具体 Arduino Board 菜单、Flash / PSRAM / USB 选项尚未在仓库中完整固化；后续若要求完全可复现构建，应把这些选项补进单独构建文档。

---

## 12. 已知限制与下一步

- 左右平移当前实机有问题，但不是现阶段 LongPet 阻塞项；
- 60 mm 轮径本身不足以把内部速度量换算成真实 cm/s，仍缺完整编码器/减速比标定；
- 精确角度转弯的误差、2°容差和 TURN_TIMEOUT 尚未完成系统验收；
- 软件 watchdog 不能代替 STBY 外部下拉、保险/限流和硬件急停；
- V2.3 已使用 normalized bbox height 设计距离滞回，现场阈值仍需按标定指南测量；
- 下一阶段优先：烧录 V2.3 -> 验证 head_offset -> 台架租约 -> 悬空轮组 -> 低速落地跟随。
