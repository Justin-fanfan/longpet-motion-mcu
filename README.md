# LongPet Motion MCU

ESP32-S3 四轮麦克纳姆底盘固件的可靠停车基线。本仓库当前只包含 MCU 固件和台架文档，不包含 LongPet、家属端、OpenCV、龙芯 GPIO/PWM 库或视觉模型。

## 工程入口

- Arduino Sketch：`xiao_che/xiao_che.ino`
- 运动控制：`xiao_che/running.h`、`xiao_che/running.cpp`
- 安全参数：`xiao_che/motion_config.h`
- 基线说明：`docs/firmware-baseline.md`
- 台架步骤：`docs/bench-test-report.md`

## 串口协议

龙芯通过 UART 以 115200 baud、8N1、无流控持续发送：

```text
dx dy area\r\n
```

`dx`、`dy` 是像素偏差，`area` 是轮廓像素面积。`area == 0` 表示目标丢失。只有完整且严格合法的一行会刷新 500 ms 链路看门时间；半包、乱码、超长行和多余字段均不会延长运动有效期。

## 重要安全边界

- 上电软件首先拉低 TB6612 的 `STBY_AB` 和 `STBY_CD`，停车清零的实际 LEDC 通道是 4～7。
- STBY 低是高阻滑行，不是主动短路刹车；驱动撤销时间与车轮完全停下时间必须分别测量。
- 链路超时、IMU 故障、转弯超时及控制周期严重超期会锁存，当前只能人工复位恢复。
- 软件不能覆盖 MCU 卡死、供电异常或复位前 GPIO 高阻窗口。正式落地仍建议 STBY 外部下拉和独立硬件急停。
- 第一次通电测试必须架空四轮，并阅读 `docs/bench-test-report.md`。其中标有“可能驱动电机”的命令不得在落地车辆上直接执行。

## 当前验证状态

- 代码分析：已完成。
- 静态源码核查：已完成。
- 编译验证：未执行；用户在本轮中明确要求停止下载库和编译。
- 烧录与硬件实测：未执行，全部等待用户配合。

已知目标版本是 Arduino ESP32 Core 2.0.14 和 `xtensa-esp32s3-elf-gcc esp-2021r2-patch5-8.4.0`。Flash、PSRAM、USB Mode、USB CDC 等实际板卡选项尚未提供，因此在补齐这些值前不能声称构建可复现。
