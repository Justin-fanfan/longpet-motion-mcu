# LongPet ESP32-S3 Motion Controller

LongPet 的通用 ESP32-S3 运动执行固件，负责头部 Servo、四轮麦克纳姆
底盘、编码器 PID、MPU6050 航向保持、UART 命令执行和安全 watchdog。本仓库
只包含 MCU 固件和台架文档，不包含 LongPet 主仓库、家属端 UI、网络接口、
OpenCV、龙芯 GPIO/PWM 库或视觉模型。

## 工程入口

- Arduino Sketch：`xiao_che/xiao_che.ino`
- 运动控制：`xiao_che/running.h`、`xiao_che/running.cpp`
- 安全参数：`xiao_che/motion_config.h`
- Motion Protocol V2：`docs/motion-protocol-v2.md`
- 基线说明：`docs/firmware-baseline.md`
- 台架步骤：`docs/bench-test-report.md`

## 当前控制模式

- `SAFE`：默认安全模式，四轮不动，头部不自动移动。
- `HEAD_ONLY`：接受 Vision `TARGET`，只驱动头部，绝不驱动底盘。
- `MANUAL`：家属端可独立控制头部和底盘；底盘 `MOVE` 必须持续刷新。
- `FOLLOW`：接收 `TARGET` 并可驱动头部的 placeholder；人物 bbox 距离阈值
  尚未重新标定，当前不自动驱动底盘。

## UART Motion Protocol V2

龙芯通过 UART 以 115200 baud、8N1、无流控持续发送：

```text
MODE MANUAL\r\n
MOVE FORWARD 20\r\n
HEAD RIGHT 20\r\n
STOP\r\n
```

完整命令、参数范围、模式合法性、legacy `dx dy area` 兼容和家属端发送
节奏见 [`docs/motion-protocol-v2.md`](docs/motion-protocol-v2.md)。协议使用
固定长度 ASCII 行，不使用 JSON 或二进制 framing；只有完整且合法的命令才
刷新链路时间。`PING` 不能刷新 MANUAL `MOVE` 或目标 TTL。

## 重要安全边界

- 上电软件首先拉低 TB6612 的 `STBY_AB` 和 `STBY_CD`，停车清零的实际 LEDC 通道是 4～7。
- STBY 低是高阻滑行，不是主动短路刹车；驱动撤销时间与车轮完全停下时间必须分别测量。
- `POWER_ON`、`STOP_COMMAND`、`TARGET_LOST`、`LINK_TIMEOUT`、`MODE_CHANGED` 和
  `MANUAL_COMMAND_TIMEOUT` 都是可恢复停车，不会锁存；IMU 故障、转弯超时和
  控制周期严重超期仍会锁存，必须复位。
- `MOVE` 是 500 ms lease；一次 MOVE 不能永久运动。连续 `ROTATE_LEFT/RIGHT`
  没有隐含 90°停止角度，只要 MOVE 新鲜就持续旋转。
- 软件不能覆盖 MCU 卡死、供电异常或复位前 GPIO 高阻窗口。正式落地仍建议 STBY 外部下拉和独立硬件急停。
- 第一次通电测试必须架空四轮，并阅读 `docs/bench-test-report.md`。其中标有“可能驱动电机”的命令不得在落地车辆上直接执行。

## 当前验证状态

- 代码分析：已按 V2 需求完成。
- 静态源码核查：已完成，仍需实际板卡验证 HEAD_ONLY 轮子硬阻断和四轮方向。
- 编译验证：`BUILD_NOT_VERIFIED`；本机未发现 Arduino CLI、PlatformIO 或已安装 ESP32 core/toolchain。
- 烧录与硬件实测：未执行，全部等待用户配合。

已知目标版本是 Arduino ESP32 Core 2.0.14 和 `xtensa-esp32s3-elf-gcc esp-2021r2-patch5-8.4.0`。Flash、PSRAM、USB Mode、USB CDC 等实际板卡选项尚未提供，因此在补齐这些值前不能声称构建可复现。
