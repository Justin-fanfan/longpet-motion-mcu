#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include <Servo.h>

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdlib.h>

#include "motion_config.h"
#include "running.h"

namespace Pins {
constexpr int AIN1 = 41;
constexpr int AIN2 = 42;
constexpr int BIN1 = 39;
constexpr int BIN2 = 38;
constexpr int CIN1 = 48;
constexpr int CIN2 = 45;
constexpr int DIN1 = 21;
constexpr int DIN2 = 20;
constexpr int PWMA = 4;
constexpr int PWMB = 5;
constexpr int PWMC = 9;
constexpr int PWMD = 10;
constexpr int STBY_AB = 40;
constexpr int STBY_CD = 47;
constexpr int ENCODER_A_A = 11;
constexpr int ENCODER_A_B = 12;
constexpr int ENCODER_B_A = 13;
constexpr int ENCODER_B_B = 14;
constexpr int ENCODER_C_A = 15;
constexpr int ENCODER_C_B = 16;
constexpr int ENCODER_D_A = 17;
constexpr int ENCODER_D_B = 18;
constexpr int IMU_SCL = 36;
constexpr int IMU_SDA = 35;
constexpr int DHT_PIN = 8;
constexpr int LINK_RX = 6;
constexpr int LINK_TX = 7;
}

enum class MotionMode : uint8_t {
    Stopped,
    Forward,
    Backward,
    LeftTurn,
    RightTurn,
    LeftShift,
    RightShift
};

enum class StopReason : uint8_t {
    PowerOn,
    AligningTarget,
    AreaHold,
    TargetNear,
    TargetLost,
    LinkTimeout,
    ImuInitFailed,
    ImuRuntimeFailed,
    TurnTimeout,
    ControlOverrun,
    TurnCompleted
};

struct TargetFrame {
    int32_t dx = 0;
    int32_t dy = 0;
    int32_t area = 0;
};

Run car(Pins::AIN1, Pins::AIN2, Pins::BIN1, Pins::BIN2,
        Pins::CIN1, Pins::CIN2, Pins::DIN1, Pins::DIN2,
        Pins::PWMA, Pins::PWMB, Pins::PWMC, Pins::PWMD,
        Pins::STBY_AB, Pins::STBY_CD);
DHT dht(Pins::DHT_PIN, DHT22);
Servo trackingServo;

MotionMode motionMode = MotionMode::Stopped;
StopReason stopReason = StopReason::PowerOn;
bool targetAvailable = false;
bool haveValidFrame = false;
bool faultLatched = false;
int servoPulseUs = MotionConfig::kServoCenterUs;

char serialLine[MotionConfig::kSerialLineCapacity] = {};
size_t serialLineLength = 0;
bool discardUntilNewline = false;
uint32_t malformedLineCount = 0;
uint32_t discardedLineCount = 0;

uint32_t lastValidFrameMs = 0;
uint32_t lastControlMs = 0;
uint32_t turnStartedMs = 0;
uint32_t lastDhtSampleMs = 0;
uint32_t lastSerialDiagnosticMs = 0;

const char* stopReasonName(StopReason reason);
const char* motionModeName(MotionMode mode);
bool elapsedAtLeast(uint32_t now, uint32_t then, uint32_t interval);
void setStopped(StopReason reason);
void latchFault(StopReason reason);
void enterMotion(MotionMode mode, uint32_t now);
bool parseInteger(const char*& cursor, int32_t& value);
bool parseTargetLine(const char* line, TargetFrame& frame);
void handleValidTarget(const TargetFrame& frame, uint32_t now);
void finishSerialLine(uint32_t now);
void pollLinkSerial(uint32_t now);
void reportSerialDiagnostics(uint32_t now);
void checkLinkTimeout(uint32_t now);
void runControlCycle(uint32_t now);
void sampleDhtWhenStopped(uint32_t now);

void setup() {
    // This is intentionally the first hardware operation.
    car.BeginSafe();

    Serial.begin(MotionConfig::kDebugBaud);
    Serial1.begin(MotionConfig::kLinkBaud, SERIAL_8N1,
                  Pins::LINK_RX, Pins::LINK_TX);
    Serial.println("[STOP] POWER_ON latched=0");

    car.EncoderSetup(Pins::ENCODER_A_A, Pins::ENCODER_A_B,
                     Pins::ENCODER_B_A, Pins::ENCODER_B_B,
                     Pins::ENCODER_C_A, Pins::ENCODER_C_B,
                     Pins::ENCODER_D_A, Pins::ENCODER_D_B);
    car.PIDSetup(60, 4.4, 1.6);

    const int servoAttachResult = trackingServo.attach(
        MotionConfig::kServoPin,
        MotionConfig::kServoMinimumPulseUs,
        MotionConfig::kServoMaximumPulseUs);
    trackingServo.writeMicroseconds(servoPulseUs);
    Serial.printf("[BOOT] SERVO_ATTACH_RESULT=%d pin=%d\n",
                  servoAttachResult, MotionConfig::kServoPin);

    if (!car.MPUSetup(Pins::IMU_SCL, Pins::IMU_SDA)) {
        latchFault(StopReason::ImuInitFailed);
    }
    dht.begin();

    const uint32_t now = millis();
    lastControlMs = now;
    lastDhtSampleMs = now;
    lastSerialDiagnosticMs = now;
    Serial.println("[BOOT] READY manual_reset_clears_faults=1");
}

void loop() {
    const uint32_t now = millis();
    pollLinkSerial(now);
    checkLinkTimeout(now);
    runControlCycle(now);
    reportSerialDiagnostics(now);
    sampleDhtWhenStopped(now);
}

const char* stopReasonName(StopReason reason) {
    switch (reason) {
    case StopReason::PowerOn: return "POWER_ON";
    case StopReason::AligningTarget: return "ALIGNING_TARGET";
    case StopReason::AreaHold: return "AREA_HOLD";
    case StopReason::TargetNear: return "TARGET_NEAR";
    case StopReason::TargetLost: return "TARGET_LOST";
    case StopReason::LinkTimeout: return "LINK_TIMEOUT";
    case StopReason::ImuInitFailed: return "IMU_INIT_FAILED";
    case StopReason::ImuRuntimeFailed: return "IMU_RUNTIME_FAILED";
    case StopReason::TurnTimeout: return "TURN_TIMEOUT";
    case StopReason::ControlOverrun: return "CONTROL_OVERRUN";
    case StopReason::TurnCompleted: return "TURN_COMPLETED";
    }
    return "UNKNOWN";
}

const char* motionModeName(MotionMode mode) {
    switch (mode) {
    case MotionMode::Stopped: return "STOPPED";
    case MotionMode::Forward: return "FORWARD";
    case MotionMode::Backward: return "BACKWARD";
    case MotionMode::LeftTurn: return "LEFT_TURN";
    case MotionMode::RightTurn: return "RIGHT_TURN";
    case MotionMode::LeftShift: return "LEFT_SHIFT";
    case MotionMode::RightShift: return "RIGHT_SHIFT";
    }
    return "UNKNOWN";
}

bool elapsedAtLeast(uint32_t now, uint32_t then, uint32_t interval) {
    return static_cast<uint32_t>(now - then) >= interval;
}

void setStopped(StopReason reason) {
    const bool changed = motionMode != MotionMode::Stopped || stopReason != reason;
    motionMode = MotionMode::Stopped;
    stopReason = reason;
    car.Stop();
    if (changed) {
        Serial.printf("[STOP] %s latched=%d\n",
                      stopReasonName(reason), faultLatched ? 1 : 0);
    }
}

void latchFault(StopReason reason) {
    faultLatched = true;
    targetAvailable = false;
    setStopped(reason);
}

void enterMotion(MotionMode mode, uint32_t now) {
    if (faultLatched || !targetAvailable || mode == MotionMode::Stopped) {
        return;
    }
    if (motionMode == mode) {
        return;
    }
    car.Stop();
    motionMode = mode;
    if (mode == MotionMode::LeftTurn || mode == MotionMode::RightTurn) {
        turnStartedMs = now;
    }
    Serial.printf("[MOTION] %s\n", motionModeName(mode));
}

bool parseInteger(const char*& cursor, int32_t& value) {
    while (*cursor != '\0' && isspace(static_cast<unsigned char>(*cursor))) {
        ++cursor;
    }
    if (*cursor == '\0') {
        return false;
    }

    errno = 0;
    char* end = nullptr;
    const long parsed = strtol(cursor, &end, 10);
    if (end == cursor || errno == ERANGE || parsed < INT32_MIN
        || parsed > INT32_MAX) {
        return false;
    }
    cursor = end;
    value = static_cast<int32_t>(parsed);
    return true;
}

bool parseTargetLine(const char* line, TargetFrame& frame) {
    const char* cursor = line;
    if (!parseInteger(cursor, frame.dx)
        || !parseInteger(cursor, frame.dy)
        || !parseInteger(cursor, frame.area)) {
        return false;
    }
    while (*cursor != '\0' && isspace(static_cast<unsigned char>(*cursor))) {
        ++cursor;
    }
    if (*cursor != '\0') {
        return false;
    }
    return frame.dx >= MotionConfig::kMinDxPixels
        && frame.dx <= MotionConfig::kMaxDxPixels
        && frame.dy >= MotionConfig::kMinDyPixels
        && frame.dy <= MotionConfig::kMaxDyPixels
        && frame.area >= MotionConfig::kMinAreaPixels
        && frame.area <= MotionConfig::kMaxAreaPixels;
}

void handleValidTarget(const TargetFrame& frame, uint32_t now) {
    haveValidFrame = true;
    lastValidFrameMs = now;

    if (faultLatched) {
        return;
    }
    if (frame.area == 0) {
        targetAvailable = false;
        setStopped(StopReason::TargetLost);
        return;
    }

    targetAvailable = true;
    if (frame.dx != 0) {
        const int64_t corrected = static_cast<int64_t>(servoPulseUs) - frame.dx;
        if (corrected < MotionConfig::kServoMinimumUs) {
            servoPulseUs = MotionConfig::kServoMinimumUs;
        } else if (corrected > MotionConfig::kServoMaximumUs) {
            servoPulseUs = MotionConfig::kServoMaximumUs;
        } else {
            servoPulseUs = static_cast<int>(corrected);
        }
        trackingServo.writeMicroseconds(servoPulseUs);
    }

    // A running turn owns the chassis until completion, timeout, or safety stop.
    if (motionMode == MotionMode::LeftTurn
        || motionMode == MotionMode::RightTurn) {
        return;
    }

    const bool horizontallyCentered =
        abs(frame.dx) < MotionConfig::kHorizontalDeadbandPixels;
    if (!horizontallyCentered) {
        setStopped(StopReason::AligningTarget);
        if (servoPulseUs == MotionConfig::kServoMaximumUs) {
            // Preserve original mapping: upper servo limit selects LeftTurn.
            enterMotion(MotionMode::LeftTurn, now);
        } else if (servoPulseUs == MotionConfig::kServoMinimumUs) {
            // Preserve original mapping: lower servo limit selects RightTurn.
            enterMotion(MotionMode::RightTurn, now);
        }
        return;
    }

    // dx == 0 reaches this branch. dy is accepted but unused by this baseline.
    if (frame.area < MotionConfig::kFarAreaExclusive) {
        enterMotion(MotionMode::Forward, now);
    } else if (frame.area <= MotionConfig::kNearAreaExclusive) {
        // The complete middle band, including 5000 and 10000, is a safe hold.
        setStopped(StopReason::AreaHold);
    } else {
        setStopped(StopReason::TargetNear);
    }
}

void finishSerialLine(uint32_t now) {
    if (discardUntilNewline) {
        discardUntilNewline = false;
        serialLineLength = 0;
        ++discardedLineCount;
        return;
    }
    if (serialLineLength == 0) {
        return;
    }

    serialLine[serialLineLength] = '\0';
    TargetFrame frame;
    if (parseTargetLine(serialLine, frame)) {
        handleValidTarget(frame, now);
    } else {
        ++malformedLineCount;
    }
    serialLineLength = 0;
}

void pollLinkSerial(uint32_t now) {
    size_t processed = 0;
    while (processed < MotionConfig::kMaxSerialBytesPerLoop
           && Serial1.available() > 0) {
        const int incoming = Serial1.read();
        if (incoming < 0) {
            break;
        }
        ++processed;
        const char value = static_cast<char>(incoming);
        if (value == '\n' || value == '\r') {
            finishSerialLine(now);
            continue;
        }
        if (discardUntilNewline) {
            continue;
        }
        if ((static_cast<unsigned char>(value) < 0x20 && value != '\t')
            || static_cast<unsigned char>(value) > 0x7e) {
            discardUntilNewline = true;
            serialLineLength = 0;
            continue;
        }
        if (serialLineLength >= MotionConfig::kSerialLineCapacity - 1) {
            discardUntilNewline = true;
            serialLineLength = 0;
            continue;
        }
        serialLine[serialLineLength++] = value;
    }
}

void reportSerialDiagnostics(uint32_t now) {
    if (!elapsedAtLeast(now, lastSerialDiagnosticMs,
                        MotionConfig::kDiagnosticRepeatMs)) {
        return;
    }
    lastSerialDiagnosticMs = now;
    if (malformedLineCount != 0 || discardedLineCount != 0) {
        Serial.printf("[LINK] rejected malformed=%lu discarded=%lu\n",
                      static_cast<unsigned long>(malformedLineCount),
                      static_cast<unsigned long>(discardedLineCount));
        malformedLineCount = 0;
        discardedLineCount = 0;
    }
}

void checkLinkTimeout(uint32_t now) {
    if (!faultLatched && haveValidFrame
        && elapsedAtLeast(now, lastValidFrameMs,
                          MotionConfig::kLinkTimeoutMs)) {
        latchFault(StopReason::LinkTimeout);
    }
}

void runControlCycle(uint32_t now) {
    const uint32_t elapsedMs = static_cast<uint32_t>(now - lastControlMs);
    if (elapsedMs < MotionConfig::kControlPeriodMs) {
        return;
    }
    lastControlMs = now;

    if (faultLatched || !targetAvailable
        || motionMode == MotionMode::Stopped) {
        car.Stop();
        return;
    }
    if (elapsedMs > MotionConfig::kControlOverrunMs) {
        latchFault(StopReason::ControlOverrun);
        return;
    }
    if ((motionMode == MotionMode::LeftTurn
         || motionMode == MotionMode::RightTurn)
        && elapsedAtLeast(now, turnStartedMs,
                          MotionConfig::kTurnTimeoutMs)) {
        latchFault(StopReason::TurnTimeout);
        return;
    }

    const float elapsedSeconds = elapsedMs / 1000.0f;
    MotionResult result = MotionResult::Running;
    switch (motionMode) {
    case MotionMode::Stopped:
        car.Stop();
        return;
    case MotionMode::Forward:
        result = car.Forward(MotionConfig::kTrackingSpeedCommand,
                             elapsedSeconds);
        break;
    case MotionMode::Backward:
        result = car.Backward(MotionConfig::kTrackingSpeedCommand,
                              elapsedSeconds);
        break;
    case MotionMode::LeftTurn:
        result = car.LeftTurn(MotionConfig::kTurnSpeedCommand,
                              MotionConfig::kTurnAngleDegrees,
                              elapsedSeconds);
        break;
    case MotionMode::RightTurn:
        result = car.RightTurn(MotionConfig::kTurnSpeedCommand,
                               MotionConfig::kTurnAngleDegrees,
                               elapsedSeconds);
        break;
    case MotionMode::LeftShift:
        result = car.LeftShift(MotionConfig::kTrackingSpeedCommand,
                               elapsedSeconds);
        break;
    case MotionMode::RightShift:
        result = car.RightShift(MotionConfig::kTrackingSpeedCommand,
                                elapsedSeconds);
        break;
    }

    if (result == MotionResult::ImuFault) {
        latchFault(StopReason::ImuRuntimeFailed);
    } else if (result == MotionResult::Completed) {
        // Require a newly received valid frame before another turn can start.
        targetAvailable = false;
        setStopped(StopReason::TurnCompleted);
    }
}

void sampleDhtWhenStopped(uint32_t now) {
    if (motionMode != MotionMode::Stopped
        || !elapsedAtLeast(now, lastDhtSampleMs,
                           MotionConfig::kDhtSamplePeriodMs)) {
        return;
    }
    lastDhtSampleMs = now;
    const float humidity = dht.readHumidity();
    const float celsius = dht.readTemperature();
    const float fahrenheit = dht.readTemperature(true);
    if (!isnan(humidity) && !isnan(celsius) && !isnan(fahrenheit)) {
        Serial.printf("[ENV] humidity=%.2f celsius=%.2f fahrenheit=%.2f\n",
                      humidity, celsius, fahrenheit);
    }
}
