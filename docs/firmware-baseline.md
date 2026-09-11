# LongPet ESP32-S3 Motion Controller 固件基线

日期：2026-09-11

> 2026-09-11 更新：本仓库已切换到 Motion Protocol V2。下方第 3、4、6、7
> 节保留为原始基线的历史记录，若与本节冲突，以本节和
> [`motion-protocol-v2.md`](motion-protocol-v2.md) 为准。

## 当前 V2 基线

### 控制模式与数据流

固件当前是 LongPet ESP32-S3 Motion Controller，执行头部 Servo、四轮麦轮、
编码器 PID、MPU6050 航向保持和 UART 命令。`ControlMode` 为：

| 模式 | 当前行为 |
|---|---|
| `SAFE` | 默认模式；四轮停止，目标和 MOVE 不会驱动执行器，头部不自动移动 |
| `HEAD_ONLY` | 合法 `TARGET` 只按 `dx` 调整头部；底盘执行路径硬阻断 |
| `MANUAL` | `HEAD` 与新鲜 `MOVE` 独立工作；MOVE 是 500 ms lease |
| `FOLLOW` | 接收 `TARGET` 并可调整头部；Tinyissimo V1.2 bbox 距离阈值未标定，底盘跟随关闭 |

实际控制链为：

```text
Serial1 bounded byte poll
  -> strict V2 line parser / legacy target parser
  -> mode legality + command freshness timestamps
  -> recoverable stop or latched fault state
  -> MANUAL-only chassis dispatch
  -> Run::Forward/Backward/LeftShift/RightShift/RotateLeft/RotateRight
  -> encoder PID + heading PID + LEDC channels 4..7 + TB6612 STBY
```

### Stop 与 Fault

以下是可恢复停车，不设置 `faultLatched`：

- `POWER_ON`
- `STOP_COMMAND`
- `TARGET_LOST`
- `LINK_TIMEOUT`
- `MODE_CHANGED`
- `MANUAL_COMMAND_TIMEOUT`
- `TARGET_TRACKING_ONLY` / `FOLLOW_CHASSIS_DISABLED`

以下继续锁存，必须 ESP reset 或 power cycle：

- `IMU_INIT_FAILED`
- `IMU_RUNTIME_FAILED`
- `CONTROL_OVERRUN`
- `TURN_TIMEOUT`

所有停车均调用 `Run::Stop()`，清零真实电机 LEDC 4、5、6、7，拉低两个 STBY，
清方向脚、编码器窗口、PID transient state 和连续旋转状态。连续旋转停止前
会执行 `syncAimToCurrentHeading()`，使后续 Forward/Backward/Shift 维持旋转后
的新航向，而不是回到旋转前的 aim。Servo 位置不因 STOP 自动回中。

### 三个独立时间戳

`lastValidLinkCommandMs` 可由任一完整且当前模式合法的命令（包括 `PING`）刷新；
`lastManualMotionCommandMs` 只由 MANUAL `MOVE` 刷新；`lastTargetCommandMs` 只由
HEAD_ONLY/FOLLOW `TARGET` 刷新。三者默认均为 500 ms。PING 不能续 MANUAL MOVE
或 TARGET lease。普通网络/UART 抖动因此只触发 recoverable stop，不会永久锁死。

### HEAD_ONLY 与 FOLLOW 安全边界

旧的“舵机到极限后自动 LeftTurn/RightTurn 90°”已从目标处理路径移除。目标
Servo 修正使用 10 像素 deadband、最大单帧 40 us correction，并始终 clamp 到
870..2270 us；`TARGET area` 不再触发任何底盘动作。旧 5000/10000 area 常量仅
保留供未来重新标定，当前 FOLLOW 不使用。

### MANUAL 与连续旋转

`MOVE FORWARD/BACKWARD/SHIFT_LEFT/SHIFT_RIGHT` 沿用原有四轮映射和 encoder PID。
新增 `Run::RotateLeft/RotateRight`，使用原地旋转轮向，不带目标角度，不调用
隐含 90°停止条件。只要 MOVE 每 100..200 ms 刷新就持续旋转；STOP、MANUAL
command timeout、LINK timeout 或模式切换立即停车。`HEAD LEFT/RIGHT/CENTER` 是
独立位置命令，允许与底盘 MOVE 同时工作。

### 协议与兼容

V2 命令完整定义见 `docs/motion-protocol-v2.md`。保留的旧行 `dx dy area` 仅按
`TARGET dx dy area` 解释，必须在 HEAD_ONLY/FOLLOW 使用，不能自动 Forward 或
自动转弯。STATUS 只写 USB debug Serial，不混入 Serial1 控制通道。

## 1. 验证结论分级

| 类型 | 本轮结果 |
|---|---|
| 代码分析 | 已完整阅读原 `xiao_che.ino`、`running.cpp`、`running.h` 并完成安全修改 |
| 静态核查 | 已检查阻塞接收、LEDC 通道、停止路径、状态转换和两个应用仓库边界 |
| 编译验证 | `BUILD_NOT_VERIFIED`：本机未发现 Arduino CLI、PlatformIO 或已安装 ESP32 core/toolchain |
| 硬件实测 | 未执行。未烧录、未打开运动串口、未运行电机命令 |

`BUILD_NOT_VERIFIED` 不代表固件已经通过编译；静态分析通过也不代表硬件方向、停车时间、IMU 符号或 PWM 资源已经验证。

## 2. 原始输入与隔离边界

原始下载文件保持不变：

| 文件 | SHA-256 |
|---|---|
| `D:\QQ\Downloads\xiao_che.ino` | `AF928C644A0E0A0271461B186CC4ED817F8C68FC750CB80565E177472DEDDFB8` |
| `D:\QQ\Downloads\running.cpp` | `6AFB2A99544D97D13A56151791F1FFE210EC16773E6ADA8D2B06BC694F95D8AF` |
| `D:\QQ\Downloads\running.h` | `88AE3EF626F123F8BD5281773181242AA704227B01C18ECF26DBF64FAD08F7F9` |

修改只位于 `D:\code\longpet-motion-mcu`。本轮未修改：

- `D:\code_qt\longpet_main\longpet`
- `D:\code\family-desktop`

没有导入两个参考压缩包中的源码，也没有加入 OpenCV、yalantinglibs 或龙芯 GPIO/PWM 库。

## 3. 历史：修改前调用链与问题（已被 V2 替代）

```text
setup
  -> DHT / Serial / Serial1 / Servo
  -> EncoderSetup
  -> MPUSetup（原代码忽略 begin 结果，并阻塞约 2 秒校准）
  -> PIDSetup
  -> 100 ms 与 3 s 硬件定时器

loop
  -> 100 ms ISR 标志 i
  -> mod 选择 Stop / Forward / Backward / Turn / Shift
  -> Run 方法 -> MPU 航向 PID -> 四电机 PID -> LEDC -> TB6612
  -> 3 s ISR 标志 ii -> DHT / 日志
  -> Serial1.readBytesUntil -> sscanf(dx, dy, area) -> 目标处理
```

原代码的电机索引是 0～3，但实际 `ledcAttachPin()` 使用通道 4～7。`Stop()` 和两个转弯结束分支错误清零 0～3，因此没有撤销真实电机 PWM。原 `Stop()` 还会反复执行 `aim -= _input[4]`。右转条件 `_input[4] > aim - 2 || _input[4] < aim + 2` 对普通数值恒真。

原目标判断整体位于 `dx != 0` 内，所以 `dx == 0 && area > 0` 不会执行远近判断；`area` 被命名为 `distance`，但实际只是轮廓像素面积。阻塞式 `readBytesUntil()`、宽松 `sscanf()` 和没有链路超时会让半包、额外字段及断联行为不可控。

## 4. 历史：修改后旧基线调用链（已被 V2 替代）

```text
setup
  -> Run::BeginSafe（首先 STBY_AB/CD=LOW，PWM 4..7=0）
  -> Serial（调试）/ Serial1（龙芯链路）
  -> Encoder / PID / Servo
  -> MPU begin + 静止偏置校准；失败则 IMU_INIT_FAILED 锁存
  -> DHT

loop（无阻塞）
  -> 最多读取 32 个 Serial1 字节
  -> 完整行严格解析与目标状态更新
  -> 每轮检查 500 ms 链路超时
  -> 基于 millis 的 100 ms 控制调度
  -> Run 方法 -> 实际 dt 的陀螺仪积分 -> PID -> LEDC 4..7 -> TB6612
  -> 仅停车时低频读取 DHT
```

`millis()` 差值全部使用无符号减法，可跨回绕。运动中一次控制间隔超过 250 ms 会锁存 `CONTROL_OVERRUN`，不会把长延迟假装成正常 100 ms；正常范围内的陀螺仪积分使用实际秒数。

## 5. 引脚与资源表

| 功能 | ESP32-S3 GPIO | 电机索引 | LEDC 通道 | 备注 |
|---|---:|---:|---:|---|
| AIN1 / AIN2 | 41 / 42 | 0 | — | A 方向 |
| PWMA | 4 | 0 | 4 | 25 kHz、10 bit |
| 编码器 A | 11 / 12 | 0 | — | HalfQuad |
| BIN1 / BIN2 | 39 / 38 | 1 | — | B 方向 |
| PWMB | 5 | 1 | 5 | 25 kHz、10 bit |
| 编码器 B | 13 / 14 | 1 | — | HalfQuad |
| CIN1 / CIN2 | 48 / 45 | 2 | — | C 方向 |
| PWMC | 9 | 2 | 6 | 25 kHz、10 bit |
| 编码器 C | 15 / 16 | 2 | — | HalfQuad |
| DIN1 / DIN2 | 21 / 20 | 3 | — | D 方向 |
| PWMD | 10 | 3 | 7 | 25 kHz、10 bit |
| 编码器 D | 17 / 18 | 3 | — | HalfQuad |
| STBY_AB | 40 | — | — | LOW 撤销 A/B 驱动 |
| STBY_CD | 47 | — | — | LOW 撤销 C/D 驱动 |
| MPU6050 SDA / SCL | 35 / 36 | — | — | I2C 地址 0x68 |
| 舵机信号 | 2 | — | 待核实 | `Servo.attach(500,2500)` |
| DHT22 | 8 | — | — | 仅停车时采样 |
| Serial1 RX / TX | 6 / 7 | — | — | 115200、8N1、无流控 |

电机的 A/B/C/D 物理方位尚无实物图，左右转轮向映射原样保留，没有猜测调换。

### 舵机 PWM 未决项

用户指定 `Servo 1.3.0`，但本轮检查到本机没有可用的 Arduino CLI/IDE、ESP32
core 或库包；提供的 `yalantinglibs.zip` 中也未找到 Servo/ESP32Servo 条目。因此
本轮无法记录“实际加载的 Servo 库路径”，也不能确认它是否会申请 LEDC 4～7。

首次编译必须打开详细输出，记录 `Multiple libraries were found for Servo.h`/`Used:` 路径，并检查该实现的通道分配源码。未确认前，不能把“代码将电机设为 4～7”当作舵机一定不冲突的证据。

## 6. 历史：V1 串口接线与语义（协议部分已被 V2 替代）

| 龙芯 40-pin 物理脚 | 方向 | ESP32-S3 |
|---|---|---|
| 8，UART2_TX | → | GPIO6，Serial1 RX |
| 10，UART2_RX | ← | GPIO7，Serial1 TX |
| GND | ↔ | GND |

物理排针号不是 GPIO 号。Linux UART 节点尚未核实，不能默认 `/dev/ttyS2`；COM9 是此前维护龙芯的电脑串口，也不能默认用于 MCU。

`Serial` 只输出调试信息；`Serial1` 只接收龙芯的 V2 命令。历史 V1 行为是：

```text
dx dy area\r\n
```

- `dx`：水平像素偏差，范围 `[-4096,4096]`；
- `dy`：垂直像素偏差，范围 `[-4096,4096]`，当前只校验和保存，不参与控制；
- `area`：轮廓像素面积，范围 `[0,16777216]`，不是物理距离；
- 只允许三个十进制整数及首尾空白；额外字段、溢出、乱码均拒绝；
- 64 字节有界行缓冲；CR、LF 或 CRLF 均结束一行，CRLF 的第二个空行会被忽略；超长或含非打印字节的行丢弃到下一个行结束符；
- 每次 `loop()` 最多消费 32 字节，支持半包、CRLF、多行积压；
- 只有完整合法行刷新链路时间。空行、半包和错误行都不刷新。

## 7. 历史：V1 目标与状态转换（已被 V2 替代）

| 输入/事件 | 行为 | 是否锁存 |
|---|---|---|
| 上电且从未收到合法帧 | `POWER_ON`，保持停车 | 否 |
| `area == 0` | `TARGET_LOST`，立即停车且不调舵机 | 否；新的合法非零 area 可恢复 |
| `area > 0` 且 `abs(dx) < 10`、`area < 5000` | 受限前进 | 否 |
| `5000 <= area <= 10000` | `AREA_HOLD`，明确停车 | 否 |
| `area > 10000` | `TARGET_NEAR`，停车 | 否 |
| 水平未居中、舵机未到限位 | 只调舵机，底盘 `ALIGNING_TARGET` | 否 |
| 水平未居中、舵机到原有上/下限 | 按原映射启动左/右转 | 否 |
| 左/右转误差进入 ±2° | 返回 `Completed` 并停车 | 否；必须再收到一帧新合法目标才可再次动作 |
| 从最后合法帧起 500 ms 无新合法帧 | `LINK_TIMEOUT`，停车 | 是，人工复位 |
| MPU 初始化/运行读取失败 | `IMU_INIT_FAILED`/`IMU_RUNTIME_FAILED` | 是，人工复位 |
| 转弯超过 3000 ms | `TURN_TIMEOUT` | 是，人工复位 |
| 运动中控制间隔超过 250 ms | `CONTROL_OVERRUN` | 是，人工复位 |

舵机水平修正和底盘面积判断已经拆开，因此 `dx == 0` 仍会判断前进/停车。5000 与 10000 均归入安全停车的中间区，避免沿用上一动作。阈值只是当前分支验证基线，必须随摄像头分辨率、裁剪和检测器重新标定。

## 8. 统一停止路径

所有安全事件最终进入 `Run::Stop()`：

1. 先将 `STBY_AB`、`STBY_CD` 拉低；
2. 将实际电机 LEDC 4、5、6、7 写 0；
3. 将八个方向脚拉低；
4. 清编码器窗口计数；
5. 清转弯活动状态；
6. 四电机 PID 和航向 PID 转 MANUAL，清输出/设定残留；
7. 不修改累计航向或 `aim`，重复调用不产生累积副作用。

从停车状态重新运动时，四个轮 PWM 都在 STBY 低期间计算并写入，最后才重新拉高 STBY；后续同方向控制周期正常更新 PWM，不反复切换 STBY。电机 PID 输出固定限制为 `[0,1023]`；负方向只作用于方向脚，写入 LEDC 前再次限制为无符号安全范围。修正后的速度请求若降到 0，会立即清该轮 PID/PWM，不沿用积分输出。航向修正限制为 ±20 个内部速度命令单位；原四轮 PID 参数 `60/4.4/1.6` 和航向参数 `0.2/0/0` 未调参。

STBY 低仅表示撤销驱动/高阻滑行。代码检测到故障至 STBY 变低的时间称“驱动撤销时间”；车轮靠惯性和摩擦完全停止是另一个机械时间，必须实测。本轮没有加入未经验证的主动刹车。

## 9. 编译环境记录

用户已确认的目标环境：

- Board：ESP32S3 Dev Module；
- Arduino ESP32 Core：2.0.14；
- 编译器：`xtensa-esp32s3-elf-gcc esp-2021r2-patch5-8.4.0`；
- Adafruit MPU6050 2.2.9；Adafruit BusIO 1.17.4；Adafruit Unified Sensor 1.1.15；
- ESP32Encoder 0.12.0；PID 1.2.0；Servo 1.3.0；DHT sensor library 1.4.7；
- Core 自带 Wire 2.0.0、SPI 2.0.0。

本机已检查常见工具入口和 `C:\Users\18214\AppData\Local\Arduino15` 缓存，但未发现 Arduino CLI、PlatformIO、Arduino IDE、ESP32 core 或编译器包，因此本轮结果为 `BUILD_NOT_VERIFIED`，没有固件产物。以下 Arduino 板卡选项也尚缺，不能自行猜测：

- CPU Frequency；
- Flash Mode、Flash Size、Partition Scheme；
- PSRAM；
- USB Mode、USB CDC On Boot、USB DFU On Boot；
- Upload Mode、Upload Speed；
- Core Debug Level；
- Arduino Runs On、Events Run On；
- Erase All Flash Before Sketch Upload、JTAG Adapter。

下次编译应先从当前能正常烧录的 Arduino IDE 导出全部选项和详细日志，再使用相同选项复现原始版与修改版，不以默认值代替缺失信息。

## 10. 未标定与已知限制

- 60 mm 轮径不足以推导 `0.151`。仍缺编码器每电机轴转计数、减速比、四倍频/半正交语义和采样周期标定；代码将其保留为“编码器换算基线”，不称为 cm/s。
- MPU Z 轴正负号、实际左右转角和 2°容差需要架空测试。
- 视觉发帧频率必须明显高于 2 Hz；建议未来 LongPet Adapter 以稳定 10 Hz 发送最新目标。
- 软件超时不能处理 MCU 死机、TB6612/STBY 线路短路或电源级故障。需要外部 STBY 下拉、保险/限流和可触达硬件急停。
- 复位到 `BeginSafe()` 执行前 GPIO 可能高阻，单靠软件不能保证这段上电窗口；必须用硬件下拉保证默认禁用。

## 11. 下一步接入建议（V2 后续工作）

先用设备树 aliases、`dmesg` 和 `/sys/class/tty` 核实龙芯 UART2 的真实 Linux 节点，再在 LongPet 中按现有分层加入 `MotionService -> SerialMotionAdapter`。页面不得直接打开串口。Vision Adapter 在 `HEAD_ONLY` 下发送 `TARGET dx dy area`；家属端在 `MANUAL` 下按 100..200 ms 周期刷新 `MOVE`，按钮释放发送 `STOP`。不要在确认本基线台架通过前接入未经验证的远程速度策略。FOLLOW 必须先完成 Tinyissimo V1.2 bbox area/distance 重新标定，再单独设计和验证自动底盘策略。
