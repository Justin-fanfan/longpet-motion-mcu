# ESP32-S3 Motion Protocol V2 台架验收报告

日期：2026-09-11

## 1. 当前状态

本文件是可复现的 V2 台架步骤，不是已通过的硬件报告。本轮未烧录、未连接
实物运动平台、未执行电机或舵机实测；下列 35 项均标记为“未验证”。编译状态
以 `firmware-baseline.md` 和最终汇报为准。

测试前置条件：四轮全部架空；附近无人员、线缆和松动物；TB6612 电机电源可立即断开；先确认 Servo 1.3.0 的实际库路径及 PWM 资源不占 LEDC 4～7。所有可能驱动底盘的用例 17..22、35 必须架空执行。

## 2. V2 验收矩阵（当前均未验证）

除特别说明外，每一项的结果栏都必须由用户在指定硬件上填写；静态代码检查
不能替代这些记录。

### A. Boot Safety

| # | 操作 | 预期 |
|---:|---|---|
| 1 | 上电，不发送 UART | 四轮不动，模式为 `SAFE`，`faultLatched=0` |
| 2 | 上电后等待 30 秒 | STBY 持续安全，实际电机 PWM 4..7 保持 0 |

### B. Protocol parsing

| # | 操作 | 预期 |
|---:|---|---|
| 3 | 发送 `PING` | 不运动；仅刷新 general link timestamp |
| 4 | 发送乱码/非打印字节并换行 | 整行拒绝，不运动、不刷新 TTL |
| 5 | 分两次发送 `MODE MANUAL\\r\\n` 或半包命令 | 仅完整行生效；半包期间不运动 |
| 6 | 发送超过 63 个有效字符后换行 | 丢弃到换行，不运动、不执行尾部内容 |
| 7 | 发送 `MOVE FORWARD 20 extra`、未知 command、溢出整数 | 整行拒绝，不运动、不刷新 TTL |

### C. Mode safety

| # | 操作 | 预期 |
|---:|---|---|
| 8 | `MODE SAFE`，再发送任意 `MOVE` | MOVE 不合法、不运动 |
| 9 | `MODE HEAD_ONLY`，持续 `TARGET 80 0 7400` | Servo 可调整；四轮始终不动 |
| 10 | HEAD_ONLY 持续极大正 `dx` | Servo 到安全一侧限位并 clamp；四轮仍不动 |
| 11 | HEAD_ONLY 持续极大负 `dx` | Servo 到另一安全侧限位并 clamp；四轮仍不动 |
| 12 | HEAD_ONLY 发送 `TARGET 0 0 0` | `targetAvailable=0`，记录 `TARGET_LOST`，不动车 |

### D. Manual head

| # | 操作 | 预期 |
|---:|---|---|
| 13 | `MODE MANUAL`，发送 `HEAD LEFT 20` | Servo 左移一个 20 us 小步 |
| 14 | 发送 `HEAD RIGHT 20` | Servo 右移一个 20 us 小步 |
| 15 | 发送 `HEAD CENTER` | Servo 回到 1570 us |
| 16 | 重复 `HEAD LEFT 100` 与 `HEAD RIGHT 100` | 分别 clamp 到 870/2270，不能越界 |

### E. Manual chassis（四轮架空）

| # | 操作 | 预期 |
|---:|---|---|
| 17 | MANUAL 持续 `MOVE FORWARD 20` | 仅按住期间前进；编码器 PID 工作 |
| 18 | MANUAL 持续 `MOVE BACKWARD 20` | 仅按住期间后退；确认物理方向 |
| 19 | MANUAL 持续 `MOVE SHIFT_LEFT 20` | 确认物理左移方向 |
| 20 | MANUAL 持续 `MOVE SHIFT_RIGHT 20` | 确认物理右移方向 |
| 21 | MANUAL 持续 `MOVE ROTATE_LEFT 20` | 持续低速原地左旋，不等待 90°完成 |
| 22 | MANUAL 持续 `MOVE ROTATE_RIGHT 20` | 持续低速原地右旋，不等待 90°完成 |

### F. Stop and recovery

| # | 操作 | 预期 |
|---:|---|---|
| 23 | 运动过程中发送 `STOP` | 立即撤销四轮驱动；Servo 不回中 |
| 24 | `STOP` 后再次持续 `MOVE FORWARD 20` | 可恢复运动，不需要 reset |

### G. Watchdog

| # | 操作 | 预期 |
|---:|---|---|
| 25 | MANUAL 以 10 Hz 刷新 `MOVE FORWARD 20` | 持续运动，不超时 |
| 26 | 停止刷新 MOVE，等待约 500 ms | `MANUAL_COMMAND_TIMEOUT`，可恢复停车 |
| 27 | 停止 MOVE，仅以 10 Hz 发送 `PING` | 仍在约 500 ms 后停车；PING 不续 MOVE lease |

### H. Recoverable link timeout

| # | 操作 | 预期 |
|---:|---|---|
| 28 | MANUAL 运动中完全断开 UART | 约 500 ms 后 `LINK_TIMEOUT latched=0`，四轮停止 |
| 29 | 恢复 UART，发送 `MODE MANUAL` 并持续有效 MOVE | 记录 `LINK_RESTORED`；无需 ESP reset 即可恢复 |

### I. Latched fault

| # | 操作 | 预期 |
|---:|---|---|
| 30 | 制造可控 IMU runtime fault | `IMU_RUNTIME_FAILED`，`faultLatched=1`，四轮停止 |
| 31 | faultLatched 后发送 MOVE/TARGET/HEAD | 全部拒绝，不运动、不改 Servo |
| 32 | faultLatched 后不 reset，继续发合法命令 | 仍锁存；只有 ESP reset/power cycle 后恢复 |

### J. Mode switching

| # | 操作 | 预期 |
|---:|---|---|
| 33 | MANUAL Forward 中发送 `MODE HEAD_ONLY` | 先立即停车，旧 Forward 不会继续 |
| 34 | HEAD_ONLY 有旧 TARGET 后发送 `MODE MANUAL` | 目标状态已清除；旧 TARGET 不会使车运动 |

### K. Head + chassis concurrent

| # | 操作 | 预期 |
|---:|---|---|
| 35 | MANUAL 持续 `MOVE FORWARD 20`，同时重复 `HEAD RIGHT 20` | 底盘持续 Forward，头部独立右移；任一流不会阻塞另一流 |

建议每项记录 USB debug `Serial` 输出、两路 STBY 电平、PWM/Servo 波形和
必要的时间戳。完成后把“未验证”替换为实测值与判定，不要把编译或静态分析
结果填写成硬件通过。

### V2 日志判读

在 USB debug `Serial` 上应观察到与当前状态一致的低频日志，例如：

```text
[STOP] POWER_ON latched=0
[MODE] MANUAL
[MOTION] FORWARD speed=20
[HEAD] RIGHT pulse=1590
[STOP] MANUAL_COMMAND_TIMEOUT latched=0
[STOP] LINK_TIMEOUT latched=0
[RECOVER] LINK_RESTORED
[FAULT] IMU_RUNTIME_FAILED
[STOP] IMU_RUNTIME_FAILED latched=1
```

HEAD_ONLY/FOLLOW 的 TARGET 不应出现任何 `[MOTION]`；HEAD_ONLY 即使 Servo
到极限也不应出现转弯日志。`STATUS` 的输出应同时反映 mode、motion、stop、
fault、target、servo 和 imu 字段。

## 附录 A：V1 日志判读（已被 V2 验收矩阵替代）

调试 `Serial` 预期包含：

```text
[STOP] POWER_ON latched=0
[BOOT] SERVO_ATTACH_RESULT=...
[BOOT] READY manual_reset_clears_faults=1
[MOTION] FORWARD
[STOP] TARGET_LOST latched=0
[STOP] LINK_TIMEOUT latched=1
[STOP] TURN_TIMEOUT latched=1
[LINK] rejected malformed=... discarded=...
```

故障锁存后继续发送合法目标也不得出现新的 `[MOTION]`；必须按 ESP32-S3 Reset 或重新上电恢复。

### 附录 A.1：V1 UART 准备（历史步骤）

龙芯 UART2 的 Linux 节点尚未知。先做只读识别，不发送数据：

```sh
cat /proc/device-tree/aliases/serial2 2>/dev/null
dmesg | grep -Ei 'tty|uart|serial'
ls -l /sys/class/tty
```

确认节点后把下文 `<UART_NODE>` 替换为实际值。不要假定 `/dev/ttyS2`，也不要使用此前维护龙芯的 COM9。

串口配置命令本身不驱动电机：

```sh
stty -F <UART_NODE> 115200 cs8 -cstopb -parenb -crtscts raw -echo
```

任何发送 `area > 0` 的命令都有可能驱动舵机或电机，以下明确标注“可能驱动电机”的命令只可在架空四轮后执行。持续运动用例必须以至少 10 Hz 重复发送，否则 500 ms 后会按设计进入 `LINK_TIMEOUT` 锁存。

### 附录 A.2：V1 验收用例（历史步骤）

| # | 用例 | 操作与预期 | 本轮结果 |
|---:|---|---|---|
| 1 | 上电静止 30 秒 | 不接龙芯发送端；四路 PWM 应为 0，两组 STBY 为 LOW，四轮无驱动 | 待用户测试 |
| 2 | 居中远目标 | **可能驱动电机**：10 Hz 持续发送 `0 0 4000\r\n`；应进入 `FORWARD` | 待用户测试 |
| 3 | 居中近目标 | **可能驱动电机**：10 Hz 持续发送 `0 0 12000\r\n`；应为 `TARGET_NEAR` 且撤销驱动 | 待用户测试 |
| 4 | 目标丢失 | 先执行用例 2，再发送 `0 0 0\r\n`；应立即记录 `TARGET_LOST`，不继续调舵机 | 待用户测试 |
| 5 | 通信中断 | 先执行用例 2，再停止发送/拔 UART；测最后合法帧至两组 STBY 下降时间，应记录 `LINK_TIMEOUT` | 待用户测试 |
| 6 | 断联恢复禁止自动运动 | 用例 5 后重新以 10 Hz 发送 `0 0 4000`；不得运动；人工复位后才可重新进入 | 待用户测试 |
| 7 | 串口鲁棒性 | 发送半包、乱码、64+ 字节行、多余字段和连续多行；错误输入不得延长 500 ms 时限或触发动作 | 待用户测试 |
| 8 | 左右转完成/超时 | **可能驱动电机**：用持续正/负 dx 逐步把舵机推至相应限位；确认原轮向映射、±2°完成和 3 秒超时锁存 | 待用户测试 |
| 9 | 连续启停 10 次 | **可能驱动电机**：在 `0 0 4000` 与 `0 0 0` 间循环；每次启动不得复用旧 PWM/转弯状态 | 待用户测试 |
| 10 | MPU 初始化失败 | 断电后断开 MPU/I2C，再上电；应记录 `IMU_INIT_FAILED` 且两组 STBY 始终 LOW | 待用户测试 |
| 11 | 停车不影响舵机 PWM | 示波器/逻辑分析仪同时观察 GPIO2、PWMA-D、STBY；停车时电机 PWM 4～7 清零，但 GPIO2 舵机脉冲不应被错误清零 | 待用户测试 |

中间区边界另测 `0 0 5000` 和 `0 0 10000`，两者都应记录 `AREA_HOLD` 并停车。`dx=0` 用例是必测项，用于证明距离判断不再依赖 `dx != 0`。

### 附录 A.3：V1 建议发送方式（历史步骤）

以下示例**可能驱动电机**，只能架空四轮执行。它持续 10 Hz 发送居中远目标，按 `Ctrl+C` 停止后应在 500 ms 左右撤销驱动并锁存：

```sh
while true; do printf '0 0 4000\r\n' > <UART_NODE>; sleep 0.1; done
```

乱码/额外字段测试本身不应驱动电机，但必须在已经确认停车的状态执行：

```sh
printf '1 2\nabc\n1 2 4000 extra\n2147483648 0 1\n' > <UART_NODE>
```

超长行应一次发送完并以换行收尾；其尾部不得被解释成下一条命令。多行合法输入可以一次写入，接收器会分多轮有界处理而不饿死控制检查。

### 附录 A.4：V1 测量记录表（历史步骤）

| 指标 | 实测值 | 判定 |
|---|---:|---|
| 上电至 STBY 明确为 LOW | 待测 | 待定 |
| `TARGET_LOST` 最后字节至 STBY LOW | 待测 | 待定 |
| 最后合法帧至 `LINK_TIMEOUT`/STBY LOW | 待测 | 目标约 500 ms，加一小段循环调度误差 |
| STBY LOW 至四轮机械完全停止 | 待测 | 只记录，不与驱动撤销时间混淆 |
| 左转完成角误差 | 待测 | 待定 |
| 右转完成角误差 | 待测 | 待定 |
| 转弯超时 | 待测 | 目标约 3000 ms |
| 连续启停旧占空比残留 | 待测 | 应无残留 |

### 附录 A.5：V1 通过门槛（历史步骤）

只有在指定 Core/库版本和完整板卡选项下编译成功，并且 11 个架空台架用例逐项记录后，才能把本基线称为“硬件验证通过”。编译成功本身不能替代 GPIO、PWM、机械停车和故障锁存实测。
