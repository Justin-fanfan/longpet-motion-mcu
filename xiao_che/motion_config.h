#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include <Arduino.h>

namespace MotionConfig {

constexpr uint32_t kDebugBaud = 115200;
constexpr uint32_t kLinkBaud = 115200;
constexpr uint32_t kControlPeriodMs = 100;
constexpr uint32_t kControlOverrunMs = 250;
constexpr uint32_t kLinkTimeoutMs = 500;
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

constexpr int32_t kHorizontalDeadbandPixels = 10;
// Temporary pixel-area thresholds; both depend on detector resolution.
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

}  // namespace MotionConfig

#endif
