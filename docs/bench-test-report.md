# ESP32-S3 Motion Protocol V2 台架验收记录

日期：2026-09-11

本文档记录当前 V2 固件的台架验收状态。它既不是“全部未验证”的旧测试计划，也不是“全部通过”的最终验收证书；
只把已经实际确认的项目标记为通过，其余继续保留为待测。

测试前置条件：

- 新的底盘动作第一次测试必须架空四轮；
- TB6612 电机电源应可快速断开；
- 控制/诊断 UART 为 `Serial1`，115200 8N1，RX=GPIO6，TX=GPIO7；
- 当前日志和 `STATUS` 都从 `Serial1` 返回；
- 左右平移已知存在实机问题，当前不作为下一阶段阻塞项。

状态说明：

- ✅ 已验证：本轮已有明确实机结果；
- ⚠️ 已发现问题：实机执行但结果不符合预期；
- ⏳ 待验证：尚未记录完整实机结论；
- ➖ 暂缓：当前版本不依赖，留待后续。

---

## 1. 当前已确认结果

### A. Boot / IMU

| # | 项目 | 结果 | 记录 |
|---:|---|---|---|
| 1 | 上电进入 `SAFE` | ✅ | 启动日志包含 `[MODE] SAFE` |
| 2 | 上电默认停车 | ✅ | 未出现上电自行驱动底盘现象 |
| 3 | I2C 设备地址 0x68 | ✅ | 可正常访问 |
| 4 | `WHO_AM_I=0x70` | ✅ | 识别为 MPU6500-compatible |
| 5 | Gyro Z 读取 | ✅ | 初始化与后续运动控制可工作 |
| 6 | bias 校准 | ✅ | 实测 `-0.018949 rad/s` |
| 7 | `IMU_INIT_FAILED` 修复 | ✅ | 启动进入 READY，不再锁存该故障 |

典型启动日志：

```text
[STOP] POWER_ON latched=0
[BOOT] SERVO_ATTACH_RESULT=0 pin=2
[IMU] detected MPU6500
[IMU] READY type=MPU6500 bias=-0.018949
[MODE] SAFE
[BOOT] READY protocol=V2 fault_reset=1
```

`SERVO_ATTACH_RESULT=0` 不应单独解释为舵机失败；本轮舵机已经通过物理动作验证。

### B. UART

| # | 项目 | 结果 | 记录 |
|---:|---|---|---|
| 8 | Motion MCU -> Host | ✅ | 启动/诊断日志可接收 |
| 9 | Host -> Motion MCU | ✅ | `MODE` / `MOVE` / `HEAD` 等可执行 |
| 10 | 双向 UART | ✅ | 使用另一块 ESP32 透明桥接打通 |

转接 ESP32 推荐透明透传：

```cpp
void loop() {
    while (Serial.available()) {
        Serial1.write(Serial.read());
    }
    while (Serial1.available()) {
        Serial.write(Serial1.read());
    }
}
```

### C. Manual chassis

| # | 操作 | 当前结果 |
|---:|---|---|
| 11 | `MOVE FORWARD <speed>` | ✅ 前进正常 |
| 12 | `MOVE BACKWARD <speed>` | ✅ 后退正常 |
| 13 | `MOVE ROTATE_LEFT <speed>` | ✅ 原地左转正常 |
| 14 | `MOVE ROTATE_RIGHT <speed>` | ✅ 原地右转正常 |
| 15 | `MOVE SHIFT_LEFT <speed>` | ⚠️ 有问题，暂缓 |
| 16 | `MOVE SHIFT_RIGHT <speed>` | ⚠️ 有问题，暂缓 |

左右平移后续重点检查：

- 麦克纳姆轮安装方向；
- A/B/C/D 电机映射；
- 四轮正反符号；
- 编码器符号；
- shift 轮速组合和 PID 输出方向。

### D. Manual head

| # | 操作 | 当前结果 |
|---:|---|---|
| 17 | `HEAD LEFT` | ✅ 正常 |
| 18 | `HEAD RIGHT` | ✅ 正常 |
| 19 | `HEAD CENTER` | ✅ 正常 |
| 20 | 实际机械总行程 | ✅ 约 120° |
| 21 | 870/2270 us 软件极限逐点边界验证 | ⏳ 尚未单独记录 |

---

## 2. 尚需补测的安全与协议矩阵

### A. Parser robustness

| # | 操作 | 预期 | 状态 |
|---:|---|---|---|
| 22 | `PING` | 不运动，仅刷新 general link | ⏳ |
| 23 | 半包命令 | 完整行前不执行 | ⏳ |
| 24 | >63 字节行 | 丢弃到行尾 | ⏳ |
| 25 | 乱码/非打印字节 | 拒绝整行 | ⏳ |
| 26 | `MOVE FORWARD 20 extra` | 拒绝整行 | ⏳ |
| 27 | 越界/溢出整数 | 拒绝整行 | ⏳ |

### B. Mode safety

| # | 操作 | 预期 | 状态 |
|---:|---|---|---|
| 28 | `MODE SAFE` 后发 `MOVE` | 不运动 | ⏳ |
| 29 | `MODE HEAD_ONLY` + `TARGET` | 可动头部，四轮不动 | ⏳ |
| 30 | HEAD_ONLY 大正/负 `dx` | Servo clamp，四轮仍不动 | ⏳ |
| 31 | `TARGET 0 0 0` | `TARGET_LOST` | ⏳ |
| 32 | MANUAL 运动中切到 HEAD_ONLY | 立即停车并清旧 motion | ⏳ |

注意：重复发送当前模式是 no-op；例如已经处于 `MANUAL` 时再次 `MODE MANUAL`，当前代码不会额外停车。

### C. STOP / recovery

| # | 操作 | 预期 | 状态 |
|---:|---|---|---|
| 33 | 运动中 `STOP` | 立即撤销四轮驱动 | ⏳ |
| 34 | STOP 后重新持续 MOVE | 可恢复，无需 reset | ⏳ |

### D. Watchdog

| # | 操作 | 预期 | 状态 |
|---:|---|---|---|
| 35 | 10 Hz 持续 MOVE | 持续运动 | ⏳ |
| 36 | 一条 MOVE 后完全静默 | 约 500 ms 停车，当前通常为 `LINK_TIMEOUT` | ⏳ |
| 37 | 停 MOVE，但持续 PING/STATUS | 约 500 ms `MANUAL_COMMAND_TIMEOUT` | ⏳ |
| 38 | UART 物理断开 | 活动底盘约 500 ms `LINK_TIMEOUT` | ⏳ |
| 39 | 恢复 UART 后重新合法控制 | 可恢复，不需 reset | ⏳ |

当前代码对活动 MANUAL 底盘先检查 general link timeout，再检查 manual lease；因此不要把“停止刷新 MOVE 后一定得到 `MANUAL_COMMAND_TIMEOUT`”写成固定结论。

### E. Latched fault

| # | 操作 | 预期 | 状态 |
|---:|---|---|---|
| 40 | 制造 IMU runtime fault | `IMU_RUNTIME_FAILED` + `fault=1` | ⏳ |
| 41 | fault 后发送 MOVE/TARGET/HEAD | 全部拒绝执行 | ⏳ |
| 42 | fault 后 reset/power-cycle | 可恢复 | ⏳ |
| 43 | `CONTROL_OVERRUN` 路径 | 锁存并停车 | ⏳ |

当前只有一个 `stopReason` 字段。严重 fault 后如果再 `STOP` 或真正切换模式，`faultLatched` 仍为 1，但 stop reason 可能被覆盖；验收时必须保存首次 `[FAULT]` 日志。

### F. Head + chassis concurrency

| # | 操作 | 预期 | 状态 |
|---:|---|---|---|
| 44 | 持续 `MOVE FORWARD` 同时发 `HEAD LEFT/RIGHT/CENTER` | 两路互不阻塞 | ⏳ |

已经准备了转接 ESP32 自动测试方式，但当前没有记录最终实测判定，因此本项暂不标 PASS。

### G. Vision / TARGET

| # | 操作 | 预期 | 状态 |
|---:|---|---|---|
| 45 | HEAD_ONLY 约 10 Hz `TARGET` | 头部连续跟踪 | ⏳ |
| 46 | Detector correction 较慢时 Tracker 更新 | Publisher 仍保持约 10 Hz 最新目标 | ⏳ |
| 47 | 目标 stale / lost | 发送 `TARGET 0 0 0`，不保留旧目标 | ⏳ |
| 48 | FOLLOW `TARGET` | 当前不得驱动底盘 | ⏳ |

---

## 3. 日志判读

所有当前 V2 日志从 `Serial1` 返回，而不是另设 USB debug `Serial`。

常见日志：

```text
[STOP] POWER_ON latched=0
[IMU] detected MPU6500
[IMU] READY type=MPU6500 bias=...
[MODE] MANUAL
[MOTION] FORWARD speed=20
[HEAD] RIGHT pulse=...
[STOP] LINK_TIMEOUT latched=0
[STOP] MANUAL_COMMAND_TIMEOUT latched=0
[RECOVER] LINK_RESTORED
[FAULT] IMU_RUNTIME_FAILED
[STATUS] mode=... motion=... stop=... fault=... target=... servo=... imu=...
```

HEAD 日志有约 2 秒限频，因此快速重复 HEAD 时不要要求每条命令都有一条 `[HEAD]`；可用 `STATUS` 检查 `servo=`。

---

## 4. 当前验收结论

当前已经足以确认：

- UART 双向链路可用；
- MPU6500-compatible 路径可用；
- 前进、后退、原地左右转可用；
- 头部舵机左右与回中可用；
- 左右平移当前存在问题，但不阻塞现阶段 HEAD_ONLY / manual 基础能力。

当前**不能**声称：

- 全部安全 watchdog 已实机逐项通过；
- HEAD_ONLY 视觉闭环已完成；
- 底盘 + Servo 并发已最终验收；
- FOLLOW 自动跟随已可用；
- 左右平移已经正常。

下一阶段优先完成龙芯 UART 与 HEAD_ONLY 视觉联调，同时逐项补齐本文件中的 ⏳ 项。
