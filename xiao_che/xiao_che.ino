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
#include <string.h>

#include "motion_config.h"
#include "head_direction.h"
#include "follow_safety.h"
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

enum class ControlMode : uint8_t {
    Safe,
    HeadOnly,
    Manual,
    Follow
};

enum class MotionMode : uint8_t {
    Stopped,
    Forward,
    Backward,
    RotateLeft,
    RotateRight,
    LeftShift,
    RightShift,
    LeftTurn,
    RightTurn
};

enum class StopReason : uint8_t {
    PowerOn,
    StopCommand,
    TargetLost,
    LinkTimeout,
    ModeChanged,
    ManualCommandTimeout,
    FollowCommandTimeout,
    TargetTrackingOnly,
    FollowChassisDisabled,
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

ControlMode controlMode = ControlMode::Safe;
MotionMode motionMode = MotionMode::Stopped;
StopReason stopReason = StopReason::PowerOn;
bool targetAvailable = false;
bool faultLatched = false;
int servoPulseUs = MotionConfig::kServoCenterUs;
TargetFrame currentTarget;
int manualMotionSpeed = 0;
bool manualMotionActive = false;
int followMotionSpeed = 0;
bool followMotionActive = false;

char serialLine[MotionConfig::kSerialLineCapacity] = {};
size_t serialLineLength = 0;
bool discardUntilNewline = false;
uint32_t malformedLineCount = 0;
uint32_t discardedLineCount = 0;

bool haveValidLinkCommand = false;
bool linkTimedOut = false;
uint32_t lastValidLinkCommandMs = 0;
uint32_t lastManualMotionCommandMs = 0;
uint32_t lastFollowMotionCommandMs = 0;
uint32_t lastTargetCommandMs = 0;
uint32_t lastControlMs = 0;
uint32_t turnStartedMs = 0;
uint32_t lastDhtSampleMs = 0;
uint32_t lastSerialDiagnosticMs = 0;
uint32_t lastHeadLogMs = 0;

const char* stopReasonName(StopReason reason);
const char* controlModeName(ControlMode mode);
const char* motionModeName(MotionMode mode);
bool elapsedAtLeast(uint32_t now, uint32_t then, uint32_t interval);
void setStopped(StopReason reason);
void latchFault(StopReason reason);
void changeControlMode(ControlMode mode);
bool startManualMotion(MotionMode mode, int speed, uint32_t now);
bool startFollowMotion(MotionMode mode, int speed, uint32_t now);
bool parseIntegerToken(const char* token, int32_t& value);
bool nextToken(const char*& cursor, char* token, size_t tokenCapacity);
bool noMoreTokens(const char* cursor);
bool parseTargetTokens(const char*& cursor, TargetFrame& frame);
bool parseLegacyTargetLine(const char* line, TargetFrame& frame);
bool parseModeToken(const char* token, ControlMode& mode);
bool parseSpeedToken(const char* token, int& speed);
bool parseHeadStepToken(const char* token, int& step);
void updateHeadFromTarget(int32_t dx, uint32_t now);
bool handleHeadCommand(const char* action, int step, uint32_t now);
void handleValidTarget(const TargetFrame& frame, uint32_t now);
bool handleCommandLine(const char* line, uint32_t now);
void markValidLinkCommand(uint32_t now);
void finishSerialLine(uint32_t now);
void pollLinkSerial(uint32_t now);
void reportSerialDiagnostics(uint32_t now);
void checkMotionTimeouts(uint32_t now);
void runControlCycle(uint32_t now);
void sampleDhtWhenStopped(uint32_t now);
void reportStatus();

void setup() {
    // This is intentionally the first hardware operation.
    car.BeginSafe();

    // GPIO6/GPIO7 Serial1 is the single bidirectional command/diagnostic UART.
    Serial1.begin(MotionConfig::kLinkBaud, SERIAL_8N1,
                  Pins::LINK_RX, Pins::LINK_TX);
    Serial1.println("[STOP] POWER_ON latched=0");

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
    Serial1.printf("[BOOT] SERVO_ATTACH_RESULT=%d pin=%d\n",
                   servoAttachResult, MotionConfig::kServoPin);

    if (!car.MPUSetup(Pins::IMU_SCL, Pins::IMU_SDA)) {
        latchFault(StopReason::ImuInitFailed);
    }
    dht.begin();

    const uint32_t now = millis();
    lastControlMs = now;
    lastDhtSampleMs = now;
    lastSerialDiagnosticMs = now;
    Serial1.println("[MODE] SAFE");
    Serial1.println("[BOOT] READY protocol=V2 fault_reset=1");
}

void loop() {
    const uint32_t now = millis();
    pollLinkSerial(now);
    checkMotionTimeouts(now);
    runControlCycle(now);
    reportSerialDiagnostics(now);
    sampleDhtWhenStopped(now);
}

const char* stopReasonName(StopReason reason) {
    switch (reason) {
    case StopReason::PowerOn: return "POWER_ON";
    case StopReason::StopCommand: return "STOP_COMMAND";
    case StopReason::TargetLost: return "TARGET_LOST";
    case StopReason::LinkTimeout: return "LINK_TIMEOUT";
    case StopReason::ModeChanged: return "MODE_CHANGED";
    case StopReason::ManualCommandTimeout: return "MANUAL_COMMAND_TIMEOUT";
    case StopReason::FollowCommandTimeout: return "FOLLOW_COMMAND_TIMEOUT";
    case StopReason::TargetTrackingOnly: return "TARGET_TRACKING_ONLY";
    case StopReason::FollowChassisDisabled: return "FOLLOW_CHASSIS_DISABLED";
    case StopReason::ImuInitFailed: return "IMU_INIT_FAILED";
    case StopReason::ImuRuntimeFailed: return "IMU_RUNTIME_FAILED";
    case StopReason::TurnTimeout: return "TURN_TIMEOUT";
    case StopReason::ControlOverrun: return "CONTROL_OVERRUN";
    case StopReason::TurnCompleted: return "TURN_COMPLETED";
    }
    return "UNKNOWN";
}

const char* controlModeName(ControlMode mode) {
    switch (mode) {
    case ControlMode::Safe: return "SAFE";
    case ControlMode::HeadOnly: return "HEAD_ONLY";
    case ControlMode::Manual: return "MANUAL";
    case ControlMode::Follow: return "FOLLOW";
    }
    return "UNKNOWN";
}

const char* motionModeName(MotionMode mode) {
    switch (mode) {
    case MotionMode::Stopped: return "STOPPED";
    case MotionMode::Forward: return "FORWARD";
    case MotionMode::Backward: return "BACKWARD";
    case MotionMode::RotateLeft: return "ROTATE_LEFT";
    case MotionMode::RotateRight: return "ROTATE_RIGHT";
    case MotionMode::LeftShift: return "SHIFT_LEFT";
    case MotionMode::RightShift: return "SHIFT_RIGHT";
    case MotionMode::LeftTurn: return "LEFT_TURN";
    case MotionMode::RightTurn: return "RIGHT_TURN";
    }
    return "UNKNOWN";
}

bool elapsedAtLeast(uint32_t now, uint32_t then, uint32_t interval) {
    return static_cast<uint32_t>(now - then) >= interval;
}

void setStopped(StopReason reason) {
    const bool changed = motionMode != MotionMode::Stopped || stopReason != reason;
    const bool wasContinuousRotation =
        motionMode == MotionMode::RotateLeft
        || motionMode == MotionMode::RotateRight;
    motionMode = MotionMode::Stopped;
    stopReason = reason;
    manualMotionActive = false;
    manualMotionSpeed = 0;
    followMotionActive = false;
    followMotionSpeed = 0;
    lastManualMotionCommandMs = 0;
    lastFollowMotionCommandMs = 0;
    turnStartedMs = 0;
    if (wasContinuousRotation) {
        car.syncAimToCurrentHeading();
    }
    car.Stop();
    if (changed) {
        Serial1.printf("[STOP] %s latched=%d\n",
                       stopReasonName(reason), faultLatched ? 1 : 0);
        if (reason == StopReason::TargetLost) {
            Serial1.println("[TARGET] LOST");
        }
    }
}

void latchFault(StopReason reason) {
    const bool firstFault = !faultLatched;
    faultLatched = true;
    targetAvailable = false;
    if (firstFault) {
        Serial1.printf("[FAULT] %s\n", stopReasonName(reason));
    }
    setStopped(reason);
}

void changeControlMode(ControlMode mode) {
    if (controlMode == mode) {
        return;
    }

    // A mode transition is a hard chassis boundary. Clear every transient
    // lease/target before exposing the new mode to the control cycle.
    setStopped(StopReason::ModeChanged);
    targetAvailable = false;
    currentTarget = TargetFrame{};
    lastManualMotionCommandMs = 0;
    lastFollowMotionCommandMs = 0;
    lastTargetCommandMs = 0;
    manualMotionActive = false;
    manualMotionSpeed = 0;
    followMotionActive = false;
    followMotionSpeed = 0;
    controlMode = mode;
    Serial1.printf("[MODE] %s\n", controlModeName(controlMode));
}

bool startManualMotion(MotionMode mode, int speed, uint32_t now) {
    if (faultLatched || controlMode != ControlMode::Manual
        || mode == MotionMode::Stopped) {
        return false;
    }

    const bool commandChanged = motionMode != mode || manualMotionSpeed != speed;
    if (commandChanged) {
        const bool wasContinuousRotation =
            motionMode == MotionMode::RotateLeft
            || motionMode == MotionMode::RotateRight;
        if (wasContinuousRotation) {
            car.syncAimToCurrentHeading();
        }
        car.Stop();
        motionMode = mode;
        Serial1.printf("[MOTION] %s speed=%d\n",
                       motionModeName(mode), speed);
    }
    manualMotionSpeed = speed;
    manualMotionActive = true;
    lastManualMotionCommandMs = now;
    return true;
}

bool startFollowMotion(MotionMode mode, int speed, uint32_t now) {
    if (faultLatched || controlMode != ControlMode::Follow) {
        return false;
    }
    if (mode == MotionMode::Stopped) {
        setStopped(StopReason::StopCommand);
        // STOP is a valid FOLLOW command but never starts a lease.
        return true;
    }
    if (mode != MotionMode::Forward
        && mode != MotionMode::RotateLeft
        && mode != MotionMode::RotateRight) {
        return false;
    }

    const bool commandChanged = motionMode != mode
        || followMotionSpeed != speed || !followMotionActive;
    if (commandChanged) {
        const bool wasContinuousRotation =
            motionMode == MotionMode::RotateLeft
            || motionMode == MotionMode::RotateRight;
        if (wasContinuousRotation) {
            car.syncAimToCurrentHeading();
        }
        car.Stop();
        motionMode = mode;
        Serial1.printf("[FOLLOW] %s speed=%d\n",
                       motionModeName(mode), speed);
    }
    manualMotionActive = false;
    manualMotionSpeed = 0;
    followMotionSpeed = speed;
    followMotionActive = true;
    lastFollowMotionCommandMs = now;
    return true;
}

bool parseIntegerToken(const char* token, int32_t& value) {
    if (token == nullptr || *token == '\0') {
        return false;
    }

    errno = 0;
    char* end = nullptr;
    const long parsed = strtol(token, &end, 10);
    if (end == token || *end != '\0' || errno == ERANGE
        || parsed < INT32_MIN || parsed > INT32_MAX) {
        return false;
    }
    value = static_cast<int32_t>(parsed);
    return true;
}

bool nextToken(const char*& cursor, char* token, size_t tokenCapacity) {
    while (*cursor == ' ') {
        ++cursor;
    }
    if (*cursor == '\0' || tokenCapacity == 0) {
        return false;
    }

    size_t length = 0;
    while (*cursor != '\0' && *cursor != ' ') {
        if (length + 1 >= tokenCapacity) {
            return false;
        }
        token[length++] = *cursor++;
    }
    token[length] = '\0';
    return true;
}

bool noMoreTokens(const char* cursor) {
    while (*cursor == ' ') {
        ++cursor;
    }
    return *cursor == '\0';
}

bool parseTargetTokens(const char*& cursor, TargetFrame& frame) {
    char token[20] = {};
    if (!nextToken(cursor, token, sizeof(token))
        || !parseIntegerToken(token, frame.dx)
        || !nextToken(cursor, token, sizeof(token))
        || !parseIntegerToken(token, frame.dy)
        || !nextToken(cursor, token, sizeof(token))
        || !parseIntegerToken(token, frame.area)
        || !noMoreTokens(cursor)) {
        return false;
    }
    return frame.dx >= MotionConfig::kMinDxPixels
        && frame.dx <= MotionConfig::kMaxDxPixels
        && frame.dy >= MotionConfig::kMinDyPixels
        && frame.dy <= MotionConfig::kMaxDyPixels
        && frame.area >= MotionConfig::kMinAreaPixels
        && frame.area <= MotionConfig::kMaxAreaPixels;
}

bool parseLegacyTargetLine(const char* line, TargetFrame& frame) {
    const char* cursor = line;
    return parseTargetTokens(cursor, frame);
}

bool parseModeToken(const char* token, ControlMode& mode) {
    if (strcmp(token, "SAFE") == 0) {
        mode = ControlMode::Safe;
    } else if (strcmp(token, "HEAD_ONLY") == 0) {
        mode = ControlMode::HeadOnly;
    } else if (strcmp(token, "MANUAL") == 0) {
        mode = ControlMode::Manual;
    } else if (strcmp(token, "FOLLOW") == 0) {
        mode = ControlMode::Follow;
    } else {
        return false;
    }
    return true;
}

bool parseSpeedToken(const char* token, int& speed) {
    int32_t parsed = 0;
    if (!parseIntegerToken(token, parsed)
        || parsed < 1 || parsed > MotionConfig::kMaximumSpeedCommand) {
        return false;
    }
    speed = static_cast<int>(parsed);
    return true;
}

bool parseHeadStepToken(const char* token, int& step) {
    int32_t parsed = 0;
    if (!parseIntegerToken(token, parsed)
        || parsed < 1 || parsed > MotionConfig::kHeadMaximumStepUs) {
        return false;
    }
    step = static_cast<int>(parsed);
    return true;
}

void updateHeadFromTarget(int32_t dx, uint32_t now) {
    const int64_t magnitude = dx < 0
        ? -static_cast<int64_t>(dx)
        : static_cast<int64_t>(dx);
    if (magnitude <= MotionConfig::kHeadTargetDeadbandPixels) {
        return;
    }

    const int64_t excess = magnitude - MotionConfig::kHeadTargetDeadbandPixels;
    int correction = 1 + static_cast<int>(
        excess / MotionConfig::kHeadTargetCorrectionDivisor);
    correction = constrain(correction, 1,
                           MotionConfig::kHeadMaximumCorrectionPerTargetUs);

    const int64_t requested = static_cast<int64_t>(servoPulseUs)
        + HeadDirection::pulseDeltaForTargetDx(dx, correction);
    const int bounded = constrain(static_cast<int>(requested),
                                  MotionConfig::kServoMinimumUs,
                                  MotionConfig::kServoMaximumUs);
    if (bounded == servoPulseUs) {
        return;
    }

    servoPulseUs = bounded;
    trackingServo.writeMicroseconds(servoPulseUs);
    if (lastHeadLogMs == 0
        || elapsedAtLeast(now, lastHeadLogMs,
                          MotionConfig::kDiagnosticRepeatMs)) {
        Serial1.printf("[HEAD] TARGET pulse=%d\n", servoPulseUs);
        lastHeadLogMs = now;
    }
}

bool handleHeadCommand(const char* action, int step, uint32_t now) {
    if (faultLatched || controlMode != ControlMode::Manual) {
        return false;
    }

    int requested = servoPulseUs;
    if (strcmp(action, "LEFT") == 0) {
        requested += HeadDirection::pulseDeltaForPhysicalLeft(step);
    } else if (strcmp(action, "RIGHT") == 0) {
        requested += HeadDirection::pulseDeltaForPhysicalRight(step);
    } else if (strcmp(action, "CENTER") == 0) {
        requested = MotionConfig::kServoCenterUs;
    } else {
        return false;
    }

    servoPulseUs = constrain(requested, MotionConfig::kServoMinimumUs,
                             MotionConfig::kServoMaximumUs);
    trackingServo.writeMicroseconds(servoPulseUs);
    if (lastHeadLogMs == 0
        || elapsedAtLeast(now, lastHeadLogMs,
                          MotionConfig::kDiagnosticRepeatMs)) {
        Serial1.printf("[HEAD] %s pulse=%d\n", action, servoPulseUs);
        lastHeadLogMs = now;
    }
    return true;
}

void handleValidTarget(const TargetFrame& frame, uint32_t now) {
    lastTargetCommandMs = now;
    currentTarget = frame;
    if (frame.area == 0) {
        targetAvailable = false;
        setStopped(StopReason::TargetLost);
        return;
    }

    targetAvailable = true;
    updateHeadFromTarget(frame.dx, now);

    // TARGET only moves the head. In FOLLOW it must not stop or renew the
    // independent chassis lease; LongPet follows with explicit FOLLOW_MOVE.
    if (controlMode == ControlMode::HeadOnly) {
        setStopped(StopReason::TargetTrackingOnly);
    }
}

void markValidLinkCommand(uint32_t now) {
    if (linkTimedOut) {
        linkTimedOut = false;
        Serial1.println("[RECOVER] LINK_RESTORED");
    }
    haveValidLinkCommand = true;
    lastValidLinkCommandMs = now;
}

void reportStatus() {
    Serial1.printf("[STATUS] mode=%s motion=%s stop=%s fault=%d target=%d "
                   "servo=%d head_offset=%d imu=%d\n",
                   controlModeName(controlMode), motionModeName(motionMode),
                   stopReasonName(stopReason), faultLatched ? 1 : 0,
                   targetAvailable ? 1 : 0, servoPulseUs,
                   HeadDirection::physicalOffsetUsForPulse(servoPulseUs),
                   car.imuReady() ? 1 : 0);
}

bool handleCommandLine(const char* line, uint32_t now) {
    char command[20] = {};
    const char* cursor = line;
    if (!nextToken(cursor, command, sizeof(command))) {
        return false;
    }

    if (strcmp(command, "PING") == 0) {
        return noMoreTokens(cursor);
    }
    if (strcmp(command, "STATUS") == 0) {
        if (!noMoreTokens(cursor)) {
            return false;
        }
        reportStatus();
        return true;
    }
    if (strcmp(command, "STOP") == 0) {
        if (!noMoreTokens(cursor)) {
            return false;
        }
        setStopped(StopReason::StopCommand);
        return true;
    }
    if (strcmp(command, "MODE") == 0) {
        char modeToken[20] = {};
        ControlMode requestedMode = ControlMode::Safe;
        if (!nextToken(cursor, modeToken, sizeof(modeToken))
            || !parseModeToken(modeToken, requestedMode)
            || !noMoreTokens(cursor)) {
            return false;
        }
        changeControlMode(requestedMode);
        return true;
    }
    if (strcmp(command, "TARGET") == 0) {
        if (faultLatched
            || (controlMode != ControlMode::HeadOnly
                && controlMode != ControlMode::Follow)) {
            return false;
        }
        TargetFrame frame;
        if (!parseTargetTokens(cursor, frame)) {
            return false;
        }
        handleValidTarget(frame, now);
        return true;
    }
    if (strcmp(command, "HEAD") == 0) {
        if (faultLatched || controlMode != ControlMode::Manual) {
            return false;
        }
        char action[20] = {};
        if (!nextToken(cursor, action, sizeof(action))) {
            return false;
        }
        if (strcmp(action, "CENTER") == 0) {
            return noMoreTokens(cursor) && handleHeadCommand(action, 0, now);
        }
        if (strcmp(action, "LEFT") != 0 && strcmp(action, "RIGHT") != 0) {
            return false;
        }
        int step = MotionConfig::kHeadDefaultStepUs;
        char stepToken[20] = {};
        if (!noMoreTokens(cursor)) {
            if (!nextToken(cursor, stepToken, sizeof(stepToken))
                || !parseHeadStepToken(stepToken, step)
                || !noMoreTokens(cursor)) {
                return false;
            }
        }
        return handleHeadCommand(action, step, now);
    }
    if (strcmp(command, "MOVE") == 0) {
        if (faultLatched || controlMode != ControlMode::Manual) {
            return false;
        }
        char direction[20] = {};
        char speedToken[20] = {};
        int speed = 0;
        if (!nextToken(cursor, direction, sizeof(direction))
            || !nextToken(cursor, speedToken, sizeof(speedToken))
            || !noMoreTokens(cursor)
            || !parseSpeedToken(speedToken, speed)) {
            return false;
        }

        MotionMode requestedMotion = MotionMode::Stopped;
        if (strcmp(direction, "FORWARD") == 0) {
            requestedMotion = MotionMode::Forward;
        } else if (strcmp(direction, "BACKWARD") == 0) {
            requestedMotion = MotionMode::Backward;
        } else if (strcmp(direction, "ROTATE_LEFT") == 0) {
            requestedMotion = MotionMode::RotateLeft;
        } else if (strcmp(direction, "ROTATE_RIGHT") == 0) {
            requestedMotion = MotionMode::RotateRight;
        } else if (strcmp(direction, "SHIFT_LEFT") == 0) {
            requestedMotion = MotionMode::LeftShift;
        } else if (strcmp(direction, "SHIFT_RIGHT") == 0) {
            requestedMotion = MotionMode::RightShift;
        } else {
            return false;
        }
        return startManualMotion(requestedMotion, speed, now);
    }
    if (strcmp(command, "FOLLOW_MOVE") == 0) {
        if (faultLatched || controlMode != ControlMode::Follow) {
            return false;
        }
        char direction[20] = {};
        if (!nextToken(cursor, direction, sizeof(direction))) {
            return false;
        }
        if (strcmp(direction, "STOP") == 0) {
            return noMoreTokens(cursor)
                && startFollowMotion(MotionMode::Stopped, 0, now);
        }
        char speedToken[20] = {};
        int speed = 0;
        if (!nextToken(cursor, speedToken, sizeof(speedToken))
            || !noMoreTokens(cursor)
            || !parseSpeedToken(speedToken, speed)) {
            return false;
        }
        MotionMode requestedMotion = MotionMode::Stopped;
        if (strcmp(direction, "FORWARD") == 0) {
            requestedMotion = MotionMode::Forward;
        } else if (strcmp(direction, "ROTATE_LEFT") == 0) {
            requestedMotion = MotionMode::RotateLeft;
        } else if (strcmp(direction, "ROTATE_RIGHT") == 0) {
            requestedMotion = MotionMode::RotateRight;
        } else {
            // BACKWARD, SHIFT and arc commands are deliberately absent.
            return false;
        }
        return startFollowMotion(requestedMotion, speed, now);
    }

    // Legacy `dx dy area` is syntax compatibility only. It follows TARGET
    // mode rules and never restores the old automatic chassis behavior.
    if (faultLatched
        || (controlMode != ControlMode::HeadOnly
            && controlMode != ControlMode::Follow)) {
        return false;
    }
    TargetFrame legacyFrame;
    if (!parseLegacyTargetLine(line, legacyFrame)) {
        return false;
    }
    handleValidTarget(legacyFrame, now);
    return true;
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
    if (handleCommandLine(serialLine, now)) {
        markValidLinkCommand(now);
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
        if (static_cast<unsigned char>(value) < 0x20
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
        Serial1.printf("[LINK] rejected malformed=%lu discarded=%lu\n",
                       static_cast<unsigned long>(malformedLineCount),
                       static_cast<unsigned long>(discardedLineCount));
        malformedLineCount = 0;
        discardedLineCount = 0;
    }
}

void checkMotionTimeouts(uint32_t now) {
    if (faultLatched) {
        return;
    }

    if ((controlMode == ControlMode::HeadOnly
         || controlMode == ControlMode::Follow) && targetAvailable
        && elapsedAtLeast(now, lastTargetCommandMs,
                          MotionConfig::kTargetTimeoutMs)) {
        targetAvailable = false;
        setStopped(StopReason::TargetLost);
        return;
    }

    if (motionMode == MotionMode::Stopped) {
        return;
    }

    // With no command at all, report the general link failure. If PING keeps
    // the link alive, the more specific manual lease below still expires.
    const bool linkExpired = !haveValidLinkCommand
        || elapsedAtLeast(now, lastValidLinkCommandMs,
                          MotionConfig::kLinkTimeoutMs);
    if (linkExpired) {
        linkTimedOut = true;
        setStopped(StopReason::LinkTimeout);
        return;
    }

    if (controlMode == ControlMode::Manual && manualMotionActive
        && elapsedAtLeast(now, lastManualMotionCommandMs,
                          MotionConfig::kManualCommandTimeoutMs)) {
        setStopped(StopReason::ManualCommandTimeout);
        return;
    }

    if (controlMode == ControlMode::Follow && followMotionActive
        && FollowSafety::leaseExpired(
            now, lastFollowMotionCommandMs,
            MotionConfig::kFollowCommandTimeoutMs)) {
        setStopped(StopReason::FollowCommandTimeout);
        return;
    }

}

void runControlCycle(uint32_t now) {
    const uint32_t elapsedMs = static_cast<uint32_t>(now - lastControlMs);
    if (elapsedMs < MotionConfig::kControlPeriodMs) {
        return;
    }
    lastControlMs = now;

    if (motionMode == MotionMode::Stopped) {
        car.Stop();
        return;
    }
    if (elapsedMs > MotionConfig::kControlOverrunMs) {
        latchFault(StopReason::ControlOverrun);
        return;
    }
    if (faultLatched) {
        car.Stop();
        return;
    }

    const bool manualDrive = controlMode == ControlMode::Manual
        && manualMotionActive;
    const bool followDrive = controlMode == ControlMode::Follow
        && followMotionActive;
    if (!manualDrive && !followDrive) {
        setStopped(controlMode == ControlMode::HeadOnly
                       ? StopReason::TargetTrackingOnly
                       : controlMode == ControlMode::Follow
                           ? StopReason::FollowChassisDisabled
                           : StopReason::ModeChanged);
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
        result = car.Forward(followDrive ? followMotionSpeed
                                         : manualMotionSpeed,
                             elapsedSeconds);
        break;
    case MotionMode::Backward:
        result = car.Backward(manualMotionSpeed,
                             elapsedSeconds);
        break;
    case MotionMode::RotateLeft:
        result = car.RotateLeft(followDrive ? followMotionSpeed
                                            : manualMotionSpeed,
                                elapsedSeconds);
        break;
    case MotionMode::RotateRight:
        result = car.RotateRight(followDrive ? followMotionSpeed
                                             : manualMotionSpeed,
                                 elapsedSeconds);
        break;
    case MotionMode::LeftTurn:
        result = car.LeftTurn(manualMotionSpeed,
                              MotionConfig::kTurnAngleDegrees,
                              elapsedSeconds);
        break;
    case MotionMode::RightTurn:
        result = car.RightTurn(manualMotionSpeed,
                               MotionConfig::kTurnAngleDegrees,
                               elapsedSeconds);
        break;
    case MotionMode::LeftShift:
        result = car.LeftShift(manualMotionSpeed,
                               elapsedSeconds);
        break;
    case MotionMode::RightShift:
        result = car.RightShift(manualMotionSpeed,
                                elapsedSeconds);
        break;
    }

    if (result == MotionResult::ImuFault) {
        latchFault(StopReason::ImuRuntimeFailed);
    } else if (result == MotionResult::Completed) {
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
        Serial1.printf("[ENV] humidity=%.2f celsius=%.2f fahrenheit=%.2f\n",
                       humidity, celsius, fahrenheit);
    }
}
