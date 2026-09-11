# LongPet 运动 MCU 串口协议与视觉模块接口使用说明

> 适用对象：LongPet 龙芯主控、视觉模块、调试用转接 ESP32  
> 运动 MCU：ESP32-S3  
> 协议版本：V2  
> 更新时间：2026-09-11

本文档偏“怎么接、怎么发、视觉端传什么”。更严格的协议语义见
[`motion-protocol-v2.md`](motion-protocol-v2.md)。

---

## 1. UART 基本配置

LongPet 运动 MCU 使用 `Serial1` 作为**唯一双向控制/诊断 UART**。

| 项目 | 配置 |
|---|---|
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验 | None |
| 停止位 | 1 |
| 流控 | None |
| MCU RX | GPIO6 |
| MCU TX | GPIO7 |
| 行结束符 | CR、LF 或 CRLF |

初始化：

```cpp
Serial1.begin(115200, SERIAL_8N1, 6, 7);
```

接线必须交叉：

```text
主控 TX  ----------> ESP32-S3 GPIO6 RX
主控 RX  <---------- ESP32-S3 GPIO7 TX
主控 GND ---------- ESP32-S3 GND
```

必须共地。

所有这些内容都从同一条 `Serial1` 返回：

- 启动日志；
- IMU 日志；
- `[MODE]` / `[MOTION]` / `[STOP]`；
- `[FAULT]` / `[LINK]`；
- `STATUS`；
- `[ENV]`。

---

## 2. 没有 USB-TTL 时：用另一块 ESP32 做串口桥

推荐使用**纯字节透明透传**，不要使用 `readBytesUntil()` 二次解析协议。

```cpp
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 6, 7);
    delay(500);
}

void loop() {
    // PC -> bridge ESP32 -> LongPet
    while (Serial.available()) {
        Serial1.write(Serial.read());
    }

    // LongPet -> bridge ESP32 -> PC
    while (Serial1.available()) {
        Serial.write(Serial1.read());
    }
}
```

如果桥接板也把 `Serial1` 映射到 RX=6 / TX=7：

```text
桥接 ESP32 GPIO7 TX  ---> LongPet GPIO6 RX
桥接 ESP32 GPIO6 RX  <--- LongPet GPIO7 TX
GND                   --- GND
```

如果桥接板是经典 ESP32-WROOM-32，一般不要使用 GPIO6~11，因为通常连接板载 SPI Flash；
可改用例如 RX=16 / TX=17 等空闲 UART GPIO。LongPet 运动 MCU 端仍保持 GPIO6/7 不变。

---

## 3. 协议格式

命令是 ASCII 单行文本：

```text
MODE MANUAL
MOVE FORWARD 30
HEAD LEFT 100
STATUS
STOP
```

推荐发送 `CRLF`：

```text
\r\n
```

固件同时接受 CR、LF 和 CRLF。

当前单行缓冲区：

```text
64 bytes
```

解析器只把普通空格当作 token 分隔符；Tab、控制字符、未知命令、额外字段、整数越界等会拒绝整行。

---

## 4. 控制模式

### SAFE

```text
MODE SAFE
```

默认安全模式，底盘禁止运动。

### HEAD_ONLY

```text
MODE HEAD_ONLY
```

用于视觉头部跟踪：

- 接受 `TARGET`；
- 允许目标驱动头部；
- 底盘执行路径硬阻止。

这是当前视觉联调推荐模式。

### MANUAL

```text
MODE MANUAL
```

用于手动/家属遥控：

- 允许 `MOVE`；
- 允许 `HEAD`；
- 头部与底盘可交错控制。

### FOLLOW

```text
MODE FOLLOW
```

当前只是未来自动跟随入口：

- 接受 `TARGET`；
- 可进行头部跟踪；
- **自动底盘跟随仍关闭**。

> 注意：只有切换到不同模式时才会触发模式切换停车。重复发送当前模式是 no-op。

---

## 5. 通用指令

### PING

```text
PING
```

作用：维持 general UART link freshness。

它不会刷新：

- MANUAL `MOVE` lease；
- `TARGET` freshness。

### STATUS

```text
STATUS
```

返回示例：

```text
[STATUS] mode=MANUAL motion=STOPPED stop=STOP_COMMAND fault=0 target=0 servo=1570 imu=1
```

字段：

| 字段 | 含义 |
|---|---|
| `mode` | 当前控制模式 |
| `motion` | 当前底盘动作 |
| `stop` | 当前记录的停车原因 |
| `fault` | 是否存在锁存故障 |
| `target` | 是否有有效视觉目标 |
| `servo` | 舵机脉宽，单位 us |
| `imu` | IMU 是否 ready |

### STOP

```text
STOP
```

立即进入统一停车路径。Servo 不自动回中。

---

## 6. 底盘指令

仅在 `MANUAL` 合法。

格式：

```text
MOVE <direction> <speed>
```

速度范围：

```text
1..100
```

它是内部控制量，不等于 cm/s。

全部方向：

```text
MOVE FORWARD 30
MOVE BACKWARD 30
MOVE ROTATE_LEFT 30
MOVE ROTATE_RIGHT 30
MOVE SHIFT_LEFT 30
MOVE SHIFT_RIGHT 30
```

截至 2026-09-11 实机：

| 指令 | 状态 |
|---|---|
| `FORWARD` | 正常 |
| `BACKWARD` | 正常 |
| `ROTATE_LEFT` | 正常 |
| `ROTATE_RIGHT` | 正常 |
| `SHIFT_LEFT` | 存在问题，暂缓 |
| `SHIFT_RIGHT` | 存在问题，暂缓 |

---

## 7. MANUAL lease 与 link timeout

当前参数：

```text
link timeout          = 500 ms
manual MOVE lease     = 500 ms
target freshness      = 500 ms
```

持续运动建议：

```text
每 100~200 ms 重发一次 MOVE
```

例如：

```text
MOVE FORWARD 30
MOVE FORWARD 30
MOVE FORWARD 30
...
```

当前代码对活动 MANUAL 底盘先检查 general link，再检查 manual lease，所以停车原因要区分：

- 一条 `MOVE` 后完全没有任何合法命令：约 500 ms 后通常是 `LINK_TIMEOUT`；
- 停止刷新 `MOVE`，但继续用 `PING` / `STATUS` 保持 general link：约 500 ms 后是 `MANUAL_COMMAND_TIMEOUT`。

两种情况都会停车。

---

## 8. 舵机指令

仅在 `MANUAL` 合法。

当前参数：

```text
GPIO2
center = 1570 us
software range = 870..2270 us
default step = 20 us
max command step = 100 us
```

命令：

```text
HEAD LEFT 100
HEAD RIGHT 100
HEAD CENTER
```

当前方向定义：

```text
LEFT  -> pulse -= step
RIGHT -> pulse += step
CENTER -> 1570 us
```

实机左右转动与回中正常，机械总活动范围约 `120°`。

HEAD 日志有约 2 秒限频；连续快速发送时不一定每条都有 `[HEAD]`，需要精确确认位置时使用 `STATUS` 查看 `servo=`。

---

## 9. 视觉模块必须传什么

视觉模块不需要发送图片，也不需要把完整 bbox 四个坐标传给 MCU。

只需要：

```text
TARGET <dx> <dy> <area>
```

例如：

```text
TARGET -32 8 7350
```

### dx

推荐定义：

```text
dx = target_center_x - image_center_x
```

因此：

```text
dx < 0   目标在画面左侧
dx = 0   水平居中
dx > 0   目标在画面右侧
```

范围：

```text
-4096..4096
```

当前头部只有水平自由度，所以 `dx` 是主要控制量。

### dy

```text
dy = target_center_y - image_center_y
```

范围：

```text
-4096..4096
```

当前只校验和保存，留给未来俯仰机构或更完整视觉逻辑。

### area

```text
area = bbox_width * bbox_height
```

单位是 `pixel²`，不是物理距离。

范围：

```text
0..16777216
```

当前配置里仍保留旧阈值：

```text
far  = 5000
near = 10000
```

但当前 FOLLOW 不使用它们驱动底盘，必须重新做摄像头/模型实机标定后才能启用自动距离控制。

---

## 10. 目标丢失

视觉侧没有可靠目标时发送：

```text
TARGET 0 0 0
```

其中 `area == 0` 表示 target lost。

如果 HEAD_ONLY/FOLLOW 下有有效 target，但超过约 500 ms 没有新的合法 `TARGET`，MCU 也会把目标判定为 stale/lost。

`PING` 不能延长目标 TTL。

---

## 11. TARGET 合法模式

`TARGET` 只在：

```text
HEAD_ONLY
FOLLOW
```

合法。

当前推荐：

```text
MODE HEAD_ONLY
TARGET -20 5 7000
TARGET -15 3 7100
TARGET -8 2 7200
...
```

这样可以先完成“视觉 -> 头部”，而不会让未标定视觉结果直接驱动底盘。

---

## 12. Legacy 三整数格式

仍兼容：

```text
<dx> <dy> <area>
```

例如：

```text
-20 5 7000
```

等价于：

```text
TARGET -20 5 7000
```

新代码统一建议使用显式 `TARGET`。

---

## 13. 视觉发送架构

当前视觉方案是：

```text
Tinyissimo Detector
+
Sparse LK Tracker
```

Detector 较慢，Tracker 更新更快，因此不要把 UART 发布绑定到 Detector 完成事件。

推荐：

```text
Detector --------\
                  > latest_target -> fixed-rate UART Publisher -> TARGET dx dy area
Tracker ---------/
```

Publisher 推荐：

```text
约 10 Hz
```

即每约 100 ms 发布一次最新且仍然新鲜的目标。

建议视觉侧维护：

```cpp
struct VisionTarget {
    bool valid;
    int dx;
    int dy;
    int area;
    uint64_t timestampMs;
};
```

伪代码：

```cpp
if (target.valid && now - target.timestampMs < 300) {
    uart.printf("TARGET %d %d %d\r\n",
                target.dx, target.dy, target.area);
} else {
    uart.print("TARGET 0 0 0\r\n");
}
```

这里 `300 ms` 是推荐的上层 freshness 门槛，不是 MCU 协议硬编码值；MCU 自身 target timeout 为 500 ms。

---

## 14. 推荐工作模式

### 视觉头部跟踪

```text
MODE HEAD_ONLY
TARGET dx dy area
TARGET dx dy area
...
```

### 手动遥控

```text
MODE MANUAL
MOVE FORWARD 30
MOVE FORWARD 30
...
HEAD LEFT 100
HEAD CENTER
STOP
```

### 紧急明确停车

```text
STOP
```

---

## 15. 当前完整指令集合

```text
PING
STATUS
STOP

MODE SAFE
MODE HEAD_ONLY
MODE MANUAL
MODE FOLLOW

TARGET <dx> <dy> <area>

HEAD LEFT [step]
HEAD RIGHT [step]
HEAD CENTER

MOVE FORWARD <speed>
MOVE BACKWARD <speed>
MOVE ROTATE_LEFT <speed>
MOVE ROTATE_RIGHT <speed>
MOVE SHIFT_LEFT <speed>
MOVE SHIFT_RIGHT <speed>
```

现阶段正式功能建议依赖：

```text
PING / STATUS / STOP
SAFE / HEAD_ONLY / MANUAL
TARGET
HEAD
FORWARD / BACKWARD / ROTATE_LEFT / ROTATE_RIGHT
```

`SHIFT_LEFT / SHIFT_RIGHT` 暂缓；`FOLLOW` 自动底盘暂未启用。

---

## 16. 当前诊断注意事项

严重 fault 锁存后，如果再发送 `STOP` 或真正切换模式，`faultLatched` 仍保持为 1，
但当前单一 `stopReason` 字段可能被新的停车原因覆盖。

因此排查严重故障时应保存首次：

```text
[FAULT] ...
```

日志，并结合 `STATUS fault=1` 判断，不要只看最后一个 `stop=`。
