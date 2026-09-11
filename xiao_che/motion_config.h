#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include <Arduino.h>

namespace MotionConfig {

constexpr uint32_t kDebugBaud = 115200;
constexpr uint32_t kLinkBaud = 115200;
constexpr uint32_t kControlPeriodMs = 100;
constexpr uint32_t kControlOverrunMs = 250;
constexpr uint32_t kLinkTimeoutMs = 500;
// These timeouts intentionally have separate names and timestamps in the
// sketch. A PING can refresh the link timeout, but never a motion lease.
constexpr uint32_t kManualCommandTimeoutMs = 500;
constexpr uint32_t kTargetTimeoutMs = 500;
constexpr uint32_t kTurnTimeoutMs = 3000;
constexpr uint32_t kDiagnosticRepeatMs = 2000;
constexpr uint32_t kDhtSamplePeriodMs = 3000;

constexpr size_t kSerialLineCapacity = 64;
constexpr size_t kMaxSerialBytesPerLoop = 32;

// Defensive protocol bounds, not camera calibration.
constexpr int32_t kMinDxPixels = -4096;
constexpr int32_t kMaxDxPixels = 4096;
constexpr int32_t kMinDyPixels = -4096;
constexpr int32_t kMaxDyPixels = 4096;
constexpr int32_t kMinAreaPixels = 0;
constexpr int32_t kMaxAreaPixels = 16777216;

// Target-to-head control is deliberately conservative: a target update can
// move the servo only a bounded amount, and small errors are ignored.
constexpr int32_t kHeadTargetDeadbandPixels = 10;
constexpr int32_t kHeadTargetCorrectionDivisor = 8;
constexpr int kHeadDefaultStepUs = 20;
constexpr int kHeadMaximumStepUs = 100;
constexpr int kHeadMaximumCorrectionPerTargetUs = 40;

// Temporary pixel-area thresholds retained for future FOLLOW calibration.
// They must not drive the chassis in the current firmware.
constexpr int32_t kFarAreaExclusive = 5000;
constexpr int32_t kNearAreaExclusive = 10000;

constexpr int kServoPin = 2;
constexpr int kServoMinimumUs = 870;
constexpr int kServoCenterUs = 1570;
constexpr int kServoMaximumUs = 2270;
constexpr int kServoMinimumPulseUs = 500;
constexpr int kServoMaximumPulseUs = 2500;

// Commands are controller units, not cm/s.
constexpr int kTrackingSpeedCommand = 20;
constexpr int kTurnSpeedCommand = 20;
constexpr int kMaximumSpeedCommand = 100;
constexpr int kTurnAngleDegrees = 90;
constexpr double kTurnToleranceDegrees = 2.0;
constexpr double kMaximumHeadingCorrection = 20.0;

constexpr uint8_t kMotorPwmChannels[4] = {4, 5, 6, 7};
constexpr uint32_t kMotorPwmFrequencyHz = 25000;
constexpr uint8_t kMotorPwmResolutionBits = 10;
constexpr uint32_t kMotorPwmMaximum = 1023;

constexpr int kImuAddress = 0x68;
constexpr size_t kImuCalibrationSamples = 100;
constexpr uint32_t kImuCalibrationDelayMs = 20;

static_assert(kFarAreaExclusive < kNearAreaExclusive,
              "far threshold must be below near threshold");
static_assert(kLinkTimeoutMs > kControlPeriodMs,
              "link timeout must allow at least one control period");
static_assert(kManualCommandTimeoutMs > kControlPeriodMs,
              "manual timeout must allow at least one control period");
static_assert(kTargetTimeoutMs > kControlPeriodMs,
              "target timeout must allow at least one control period");
static_assert(kHeadMaximumStepUs >= kHeadDefaultStepUs,
              "maximum head step must cover the default step");

}  // namespace MotionConfig

#endif
