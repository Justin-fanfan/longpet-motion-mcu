# LongPet Motion Protocol V2

## 1. Scope

本文档描述 LongPet 主控与 ESP32-S3 Motion Controller 之间的 ASCII UART 协议。
运动 MCU 负责头部 Servo、四轮麦克纳姆底盘、编码器 PID、IMU 航向反馈和安全 watchdog；
网络 API、视觉检测/跟踪算法和 LongPet UI 不在本协议内。

UART：

```text
115200 baud, 8N1, no flow control
ESP32-S3 Serial1 RX = GPIO6
ESP32-S3 Serial1 TX = GPIO7
```

所有控制命令、状态返回、启动日志和诊断日志都走同一条双向 `Serial1`。

每条命令是一行 ASCII，以 `CR`、`LF` 或 `CRLF` 结束。接收器使用固定 64-byte 缓冲，
支持半包和多行积压；每次 `loop()` 最多处理 32 字节。只有完整、格式正确且在当前模式合法的命令才算有效链路流量。

解析器只把普通空格当作 token 分隔符。Tab、其他控制字符、非 ASCII 字节、额外字段、未知命令、整数溢出或越界值都会拒绝整行；被拒绝的行不会刷新任何 timeout。

---

## 2. Control modes

| Mode | Purpose | Chassis behavior |
|---|---|---|
| `SAFE` | 默认安全态 | 底盘保持停止 |
| `HEAD_ONLY` | 视觉头部跟踪 | `TARGET` 可动头部，轮子硬阻止 |
| `MANUAL` | 手动/家属遥控 | `HEAD` 与持续刷新的 `MOVE` 独立工作 |
| `FOLLOW` | V2.3 人物跟随 | `TARGET` 只跟头；底盘只执行带独立租约的 `FOLLOW_MOVE` |

上电后初始模式为 `SAFE`。

只有**切换到不同模式**时才会：

- 先停车；
- 清 active motion；
- 清 MANUAL motion lease；
- 清 target freshness；
- 清瞬态运动状态。

重复发送当前模式，例如车辆已经处于 `MANUAL` 时再次发送：

```text
MODE MANUAL
```

当前实现是 no-op，不会额外停车或清状态。

模式切换不会清除 `faultLatched`。

---

## 3. Commands

### 3.1 Mode / stop / diagnostics

```text
MODE SAFE
MODE HEAD_ONLY
MODE MANUAL
MODE FOLLOW

PING
STOP
STATUS
```

`STOP` 立即进入统一停车路径：撤销 TB6612 STBY、清电机 PWM 和当前 chassis command；Servo 保持当前角度，不自动回中。

`PING` 只证明 general UART link 仍然活着。它会刷新 general link timestamp，但不会刷新：

- MANUAL `MOVE` lease；
- `TARGET` freshness。

`STATUS` 从 **Serial1** 返回：

```text
[STATUS] mode=<...> motion=<...> stop=<...> fault=<0|1> target=<0|1> servo=<us> head_offset=<us> imu=<0|1>
```

例如：

```text
[STATUS] mode=MANUAL motion=STOPPED stop=STOP_COMMAND fault=0 target=0 servo=1570 head_offset=0 imu=1
```

### 3.2 Vision target

```text
TARGET <dx> <dy> <area>
```

范围：

| Parameter | Range | Meaning |
|---|---:|---|
| `dx` | `-4096..4096` | 目标中心相对画面中心的水平偏差 |
| `dy` | `-4096..4096` | 垂直偏差；当前只校验和保存 |
| `area` | `0..16777216` | bbox 像素面积 |

推荐视觉侧定义：

```text
dx   = target_center_x - image_center_x
dy   = target_center_y - image_center_y
area = bbox_width * bbox_height
```

`TARGET` 只在 `HEAD_ONLY` / `FOLLOW` 合法。

`area == 0` 表示目标丢失：`targetAvailable=false`，记录 `TARGET_LOST`。

非零目标会更新 target freshness，并按 `dx` 调整头部。当前头部修正：

- deadband：10 px；
- 单次最大 correction：40 us；
- Servo clamp：870..2270 us；
- `dx < 0` 时按“物理向左”映射调整，`dx > 0` 时按“物理向右”映射调整；
- 当前装配 `kServoPhysicalLeftPulseSign=+1`，因此物理向左为 pulse 增大、物理向右为 pulse 减小。

`HEAD_ONLY` 中 `TARGET` 永远不能进入底盘执行路径。

`FOLLOW` 不会依据像素 `area` 自主决策底盘。距离和头身策略位于 LongPet，MCU 只执行显式
`FOLLOW_MOVE`。5000/10000 是未使用的旧阈值。

### 3.3 Head commands

```text
HEAD LEFT [step]
HEAD RIGHT [step]
HEAD CENTER
```

只在 `MANUAL` 合法。

`step`：

```text
1..100 us
默认 20 us
```

当前方向定义：

```text
LEFT  -> physical left  -> pulse += step
RIGHT -> physical right -> pulse -= step
CENTER -> 1570 us
```

`HEAD` 和 `TARGET` 均调用 `head_direction.h` 中的同一物理方向映射。若未来更换舵机或改变安装方向，
只调整 `motion_config.h` 的 `kServoPhysicalLeftPulseSign`，不要在协议解析或 UI 中交换 LEFT/RIGHT。

位置始终 clamp 到：

```text
870..2270 us
```

HEAD 是位置步进，不是 lease。停止发送后 Servo 保持最后位置，没有 head watchdog。

注意：HEAD 日志按 `kDiagnosticRepeatMs=2000 ms` 限频，因此连续发送 HEAD 时不一定每条都立即出现 `[HEAD]`；需要精确确认位置时使用 `STATUS` 查看 `servo=`。

### 3.4 Manual chassis commands

```text
MOVE FORWARD <speed>
MOVE BACKWARD <speed>
MOVE ROTATE_LEFT <speed>
MOVE ROTATE_RIGHT <speed>
MOVE SHIFT_LEFT <speed>
MOVE SHIFT_RIGHT <speed>
```

只在 `MANUAL` 合法。

`speed`：

```text
1..100
```

越界值直接拒绝，不会自动 clamp。

每条合法 `MOVE` 都替换当前 manual chassis command，并刷新 `lastManualMotionCommandMs`。
`ROTATE_LEFT/RIGHT` 是持续原地旋转，不带隐式 90°目标。

当前实机状态：

```text
FORWARD       verified OK
BACKWARD      verified OK
ROTATE_LEFT   verified OK
ROTATE_RIGHT  verified OK
SHIFT_LEFT    known issue / deferred
SHIFT_RIGHT   known issue / deferred
```

左右平移协议保留，但现阶段不要作为 LongPet 必需功能。

### 3.5 FOLLOW chassis commands

```text
FOLLOW_MOVE FORWARD <speed>
FOLLOW_MOVE ROTATE_LEFT <speed>
FOLLOW_MOVE ROTATE_RIGHT <speed>
FOLLOW_MOVE STOP
```

只在 `MODE FOLLOW` 合法。没有 BACKWARD、SHIFT 或弧线命令；V2.3 策略只允许低速前进和原地对齐。
每条非 STOP `FOLLOW_MOVE` 都刷新独立的 `lastFollowMotionCommandMs`，默认租约 500 ms。以下命令均不续租：

- `PING`；
- `STATUS`；
- `TARGET`；
- MANUAL `MOVE`。

租约到期产生 `FOLLOW_COMMAND_TIMEOUT` 并停车。`FOLLOW_MOVE STOP` 立即停车并清租约。

---

## 4. Watchdogs and timestamps

固件维护三个独立 timestamp：

| Timestamp | Refreshed by | Timeout |
|---|---|---:|
| `lastValidLinkCommandMs` | 任意被接受的合法命令，包括 `PING` / `STATUS` / `MODE` | 500 ms |
| `lastManualMotionCommandMs` | 仅 MANUAL `MOVE` | 500 ms |
| `lastTargetCommandMs` | 仅 HEAD_ONLY/FOLLOW `TARGET` | 500 ms |
| `lastFollowMotionCommandMs` | 仅 FOLLOW `FOLLOW_MOVE` | 500 ms |

### 4.1 MANUAL 实际 timeout 顺序

当前 `checkMotionTimeouts()` 对活动底盘的顺序是：

1. general link timeout；
2. MANUAL motion lease timeout。

因此：

- 只发一次 `MOVE`，之后完全没有任何合法命令：约 500 ms 后通常得到 `LINK_TIMEOUT`；
- 持续发 `PING` / `STATUS` 保持 general link 新鲜，但不再发 `MOVE`：约 500 ms 后得到 `MANUAL_COMMAND_TIMEOUT`；
- 两种情况都会停车。

这意味着 `PING` **不能**延长运动，只会让停车原因更具体地落到 MANUAL lease。

### 4.2 TARGET timeout

HEAD_ONLY/FOLLOW 下，如果当前有有效目标且 500 ms 内没有新的合法 `TARGET`：

```text
TARGET_LOST
```

`PING` 不能刷新 target freshness。

### 4.3 推荐发送频率

MANUAL 持续运动：

```text
MOVE every 100..200 ms
```

视觉目标：

```text
TARGET about 10 Hz
FOLLOW_MOVE with every newly evaluated target observation, about 7 Hz
```

按钮释放或上层需要明确停车时应立即发送：

```text
STOP
```

---

## 5. Stop and fault semantics

### 5.1 Recoverable stops

以下不会设置 `faultLatched`：

- `POWER_ON`
- `STOP_COMMAND`
- `TARGET_LOST`
- `LINK_TIMEOUT`
- `MODE_CHANGED`
- `MANUAL_COMMAND_TIMEOUT`
- `FOLLOW_COMMAND_TIMEOUT`
- `TARGET_TRACKING_ONLY`
- `FOLLOW_CHASSIS_DISABLED`

### 5.2 Latched faults

以下会锁存，直到 ESP reset / power cycle：

- `IMU_INIT_FAILED`
- `IMU_RUNTIME_FAILED`
- `CONTROL_OVERRUN`
- `TURN_TIMEOUT`

faultLatched 后：

- `MOVE` / `TARGET` / active `HEAD` 被拒绝；
- `PING` / `STATUS` / `STOP` / `MODE` 仍能被解析；
- 改模式不会清 fault latch。

### 5.3 当前诊断限制

当前固件只有一个 `stopReason` 字段。发生锁存故障后，如果再发送 `STOP` 或切换到另一个模式，
`faultLatched` 仍保持为 1，但 `stopReason` 可能被新的可恢复停车原因覆盖。

因此排查严重故障时应保留并优先查看首次：

```text
[FAULT] ...
```

日志，不要只根据之后的 `STATUS stop=` 推断根因。

---

## 6. Legacy compatibility

旧格式仍兼容：

```text
<dx> <dy> <area>
```

它严格等价于：

```text
TARGET <dx> <dy> <area>
```

也只在 `HEAD_ONLY` / `FOLLOW` 合法。

旧格式不再：

- 自动前进；
- 在 Servo 到限位时自动 90°转弯；
- 使用 5000/10000 area 阈值驱动底盘。

新代码推荐统一使用显式 `TARGET`。

---

## 7. Sending guide

### 7.1 Family/manual control

```text
MODE MANUAL
```

按住 Forward 时约 10 Hz 重发：

```text
MOVE FORWARD 20
MOVE FORWARD 20
MOVE FORWARD 20
```

释放：

```text
STOP
```

头部与底盘可以交错：

```text
MOVE FORWARD 20
HEAD RIGHT 20
MOVE FORWARD 20
```

### 7.2 Head-only vision

```text
MODE HEAD_ONLY
TARGET -85 12 7400
TARGET -20 10 7300
TARGET 0 8 0
```

### 7.3 Person follow

```text
MODE FOLLOW
TARGET 15 0 7400
FOLLOW_MOVE ROTATE_RIGHT 10
TARGET 2 0 7600
FOLLOW_MOVE FORWARD 12
TARGET 0 0 0
FOLLOW_MOVE STOP
```

LongPet 必须对每个新鲜 observation 重新评估策略，不能靠重复旧帧刷新 `FOLLOW_MOVE`。头偏时先原地转向；
只有物理头偏进入回中滞回且距离为 FAR 时才前进。MCU 不参与 bbox 距离分类。

---

## 8. Current hardware verification

截至 2026-09-11：

- 双向 UART：通过；
- MPU6500-compatible 检测、Gyro Z、bias：通过；
- Forward / Backward / Rotate Left / Rotate Right：通过；
- Shift Left / Shift Right：实机存在问题，暂缓；
- Servo Left / Right / Center：通过，机械总活动范围约 120°；
- V2.2 HEAD_ONLY：用户已确认通过；
- V2.3 FOLLOW_MOVE / 独立租约 / 头身与距离闭环：软件已完成，硬件待用户测试。
