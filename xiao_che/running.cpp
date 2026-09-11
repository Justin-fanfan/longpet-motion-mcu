#include "running.h"

#include "motion_config.h"

#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <algorithm>
#include <cmath>

namespace {
constexpr double kEncoderScaleBaseline = 0.151;
constexpr double kRadiansToDegrees = 57.29577951308232;
}

Run::Run(int ain1, int ain2, int bin1, int bin2, int cin1, int cin2,
         int din1, int din2, int pwma, int pwmb, int pwmc, int pwmd,
         int stbyab, int stbycd)
    : _ain1(ain1), _ain2(ain2), _bin1(bin1), _bin2(bin2),
      _cin1(cin1), _cin2(cin2), _din1(din1), _din2(din2),
      _pwma(pwma), _pwmb(pwmb), _pwmc(pwmc), _pwmd(pwmd),
      _stbyab(stbyab), _stbycd(stbycd) {
}

void Run::BeginSafe() {
    // Disable both TB6612 groups before configuring any motor output.
    pinMode(_stbyab, OUTPUT);
    pinMode(_stbycd, OUTPUT);
    digitalWrite(_stbyab, LOW);
    digitalWrite(_stbycd, LOW);

    const int directionPins[] = {
        _ain1, _ain2, _bin1, _bin2, _cin1, _cin2, _din1, _din2
    };
    for (int pin : directionPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    const int pwmPins[] = {_pwma, _pwmb, _pwmc, _pwmd};
    for (size_t index = 0; index < 4; ++index) {
        const uint8_t channel = MotionConfig::kMotorPwmChannels[index];
        ledcSetup(channel, MotionConfig::kMotorPwmFrequencyHz,
                  MotionConfig::kMotorPwmResolutionBits);
        ledcAttachPin(pwmPins[index], channel);
        ledcWrite(channel, 0);
    }
    _resetPidState();
}

void Run::_disableDrivers() {
    digitalWrite(_stbyab, LOW);
    digitalWrite(_stbycd, LOW);
}

void Run::_enableDrivers() {
    digitalWrite(_stbyab, HIGH);
    digitalWrite(_stbycd, HIGH);
}

void Run::_clearMotorOutputs() {
    for (uint8_t channel : MotionConfig::kMotorPwmChannels) {
        ledcWrite(channel, 0);
    }
    digitalWrite(_ain1, LOW);
    digitalWrite(_ain2, LOW);
    digitalWrite(_bin1, LOW);
    digitalWrite(_bin2, LOW);
    digitalWrite(_cin1, LOW);
    digitalWrite(_cin2, LOW);
    digitalWrite(_din1, LOW);
    digitalWrite(_din2, LOW);
}

void Run::_resetPidState() {
    for (size_t index = 0; index < 4; ++index) {
        pid[index].SetMode(MANUAL);
        _input[index] = 0;
        _output[index] = 0;
        _setpoint[index] = 0;
    }
    pid[4].SetMode(MANUAL);
    _output[4] = 0;
    _setpoint[4] = _input[4];
}

void Run::_setMotor(int pin1, int pin2, int motorIndex,
                    int expect, double correction) {
    if (motorIndex < 0 || motorIndex >= 4) {
        return;
    }

    double measured = static_cast<double>(enc[motorIndex].getCount());
    enc[motorIndex].clearCount();
    measured *= kEncoderScaleBaseline;

    const bool reverse = expect < 0;
    const int magnitude = std::min(std::abs(expect),
                                   MotionConfig::kMaximumSpeedCommand);
    if (reverse) {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        measured = -measured;
    } else {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    }

    const double requested = std::max(0.0, std::min(
        static_cast<double>(MotionConfig::kMaximumSpeedCommand),
        static_cast<double>(magnitude) + correction));
    _input[motorIndex] = measured;
    _setpoint[motorIndex] = requested;
    if (requested <= 0.0) {
        pid[motorIndex].SetMode(MANUAL);
        _output[motorIndex] = 0;
        ledcWrite(MotionConfig::kMotorPwmChannels[motorIndex], 0);
        return;
    }
    pid[motorIndex].SetOutputLimits(0, MotionConfig::kMotorPwmMaximum);
    pid[motorIndex].SetMode(AUTOMATIC);
    pid[motorIndex].Compute();

    const double boundedOutput = std::max(0.0, std::min(
        static_cast<double>(MotionConfig::kMotorPwmMaximum),
        _output[motorIndex]));
    ledcWrite(MotionConfig::kMotorPwmChannels[motorIndex],
              static_cast<uint32_t>(boundedOutput));
}

bool Run::_updateHeading(int angle, float elapsedSeconds) {
    if (!_imuReady || !std::isfinite(elapsedSeconds)
        || elapsedSeconds <= 0.0f) {
        return false;
    }
    if (!MPU.getEvent(&a, &g, &t)) {
        _imuReady = false;
        return false;
    }

    const double correctedRate = g.gyro.z - Bias;
    if (correctedRate >= 0.005 || correctedRate <= -0.015) {
        // Integrate with the measured interval, not an assumed 100 ms.
        _input[4] += correctedRate * elapsedSeconds * kRadiansToDegrees;
    }
    _setpoint[4] = static_cast<double>(angle);
    pid[4].SetMode(AUTOMATIC);
    pid[4].Compute();
    return true;
}

void Run::PIDSetup(double Kp, double Ki, double Kd) {
    for (size_t index = 0; index < 4; ++index) {
        pid[index].SetTunings(Kp, Ki, Kd);
        pid[index].SetOutputLimits(0, MotionConfig::kMotorPwmMaximum);
        pid[index].SetSampleTime(MotionConfig::kControlPeriodMs);
        pid[index].SetMode(MANUAL);
    }
    pid[4].SetOutputLimits(-MotionConfig::kMaximumHeadingCorrection,
                           MotionConfig::kMaximumHeadingCorrection);
    pid[4].SetSampleTime(MotionConfig::kControlPeriodMs);
    pid[4].SetMode(MANUAL);
}

bool Run::MPUSetup(int SCL, int SDA) {
    Wire.begin(SDA, SCL);
    if (!MPU.begin(MotionConfig::kImuAddress, &Wire)) {
        _imuReady = false;
        return false;
    }

    Bias = 0;
    for (size_t sample = 0; sample < MotionConfig::kImuCalibrationSamples;
         ++sample) {
        if (!MPU.getEvent(&a, &g, &t)) {
            _imuReady = false;
            return false;
        }
        Bias += g.gyro.z;
        delay(MotionConfig::kImuCalibrationDelayMs);
    }
    Bias /= MotionConfig::kImuCalibrationSamples;
    _input[4] = 0;
    aim = 0;
    _imuReady = true;
    return true;
}

void Run::EncoderSetup(int A_A, int A_B, int B_A, int B_B,
                       int C_A, int C_B, int D_A, int D_B) {
    enc[0].attachHalfQuad(A_A, A_B);
    enc[1].attachHalfQuad(B_A, B_B);
    enc[2].attachHalfQuad(C_A, C_B);
    enc[3].attachHalfQuad(D_A, D_B);
    for (ESP32Encoder& encoder : enc) {
        encoder.setFilter(250);
        encoder.clearCount();
    }
}

void Run::Stop() {
    // STBY low means coast/high impedance, not active short-circuit braking.
    _disableDrivers();
    _clearMotorOutputs();
    for (ESP32Encoder& encoder : enc) {
        encoder.clearCount();
    }
    _turnActive = false;
    _turnDirection = 0;
    _resetPidState();
}

MotionResult Run::Forward(int speed, float elapsedSeconds) {
    if (!_updateHeading(aim, elapsedSeconds)) {
        Stop();
        return MotionResult::ImuFault;
    }
    _setMotor(_ain1, _ain2, 0, speed, -_output[4]);
    _setMotor(_bin1, _bin2, 1, speed, _output[4]);
    _setMotor(_cin1, _cin2, 2, speed, -_output[4]);
    _setMotor(_din1, _din2, 3, speed, _output[4]);
    _enableDrivers();
    return MotionResult::Running;
}

MotionResult Run::Backward(int speed, float elapsedSeconds) {
    if (!_updateHeading(aim, elapsedSeconds)) {
        Stop();
        return MotionResult::ImuFault;
    }
    _setMotor(_ain1, _ain2, 0, -speed, _output[4]);
    _setMotor(_bin1, _bin2, 1, -speed, -_output[4]);
    _setMotor(_cin1, _cin2, 2, -speed, _output[4]);
    _setMotor(_din1, _din2, 3, -speed, -_output[4]);
    _enableDrivers();
    return MotionResult::Running;
}

MotionResult Run::_rotate(int direction, int speed, float elapsedSeconds) {
    // Keep integrating the IMU while rotating so a later manual translation
    // can adopt the heading reached by the user. The heading PID is not used
    // as a stop condition for this command.
    const int currentHeading = static_cast<int>(std::lround(_input[4]));
    if (!_updateHeading(currentHeading, elapsedSeconds)) {
        Stop();
        return MotionResult::ImuFault;
    }

    if (direction < 0) {
        // Preserve the original A/B/C/D wheel map for left rotation.
        _setMotor(_ain1, _ain2, 0, speed, 0);
        _setMotor(_bin1, _bin2, 1, -speed, 0);
        _setMotor(_cin1, _cin2, 2, speed, 0);
        _setMotor(_din1, _din2, 3, -speed, 0);
    } else {
        // Preserve the original A/B/C/D wheel map for right rotation.
        _setMotor(_ain1, _ain2, 0, -speed, 0);
        _setMotor(_bin1, _bin2, 1, speed, 0);
        _setMotor(_cin1, _cin2, 2, -speed, 0);
        _setMotor(_din1, _din2, 3, speed, 0);
    }
    _enableDrivers();
    return MotionResult::Running;
}

MotionResult Run::RotateLeft(int speed, float elapsedSeconds) {
    return _rotate(-1, speed, elapsedSeconds);
}

MotionResult Run::RotateRight(int speed, float elapsedSeconds) {
    return _rotate(1, speed, elapsedSeconds);
}

MotionResult Run::_turn(int direction, int speed, int angle,
                        float elapsedSeconds) {
    if (!_turnActive) {
        const int boundedAngle = std::max(0, std::min(std::abs(angle), 360));
        aim = static_cast<int>(std::lround(_input[4]))
            + direction * boundedAngle;
        _turnActive = true;
        _turnDirection = direction;
    } else if (_turnDirection != direction) {
        Stop();
        return MotionResult::Completed;
    }

    if (!_updateHeading(aim, elapsedSeconds)) {
        Stop();
        return MotionResult::ImuFault;
    }
    const double angleError = static_cast<double>(aim) - _input[4];
    const bool reachedOrPassed = direction < 0
        ? angleError >= -MotionConfig::kTurnToleranceDegrees
        : angleError <= MotionConfig::kTurnToleranceDegrees;
    if (reachedOrPassed) {
        Stop();
        return MotionResult::Completed;
    }

    if (direction < 0) {
        // Preserve the original A/B/C/D wheel map for left turn.
        _setMotor(_ain1, _ain2, 0, speed, 0);
        _setMotor(_bin1, _bin2, 1, -speed, 0);
        _setMotor(_cin1, _cin2, 2, speed, 0);
        _setMotor(_din1, _din2, 3, -speed, 0);
    } else {
        // Preserve the original A/B/C/D wheel map for right turn.
        _setMotor(_ain1, _ain2, 0, -speed, 0);
        _setMotor(_bin1, _bin2, 1, speed, 0);
        _setMotor(_cin1, _cin2, 2, -speed, 0);
        _setMotor(_din1, _din2, 3, speed, 0);
    }
    _enableDrivers();
    return MotionResult::Running;
}

MotionResult Run::LeftTurn(int speed, int angle, float elapsedSeconds) {
    return _turn(-1, speed, angle, elapsedSeconds);
}

MotionResult Run::RightTurn(int speed, int angle, float elapsedSeconds) {
    return _turn(1, speed, angle, elapsedSeconds);
}

MotionResult Run::LeftShift(int speed, float elapsedSeconds) {
    if (!_updateHeading(aim, elapsedSeconds)) {
        Stop();
        return MotionResult::ImuFault;
    }
    _setMotor(_ain1, _ain2, 0, speed, -_output[4]);
    _setMotor(_bin1, _bin2, 1, -speed, -_output[4]);
    _setMotor(_cin1, _cin2, 2, -speed, _output[4]);
    _setMotor(_din1, _din2, 3, speed, _output[4]);
    _enableDrivers();
    return MotionResult::Running;
}

MotionResult Run::RightShift(int speed, float elapsedSeconds) {
    if (!_updateHeading(aim, elapsedSeconds)) {
        Stop();
        return MotionResult::ImuFault;
    }
    _setMotor(_ain1, _ain2, 0, -speed, _output[4]);
    _setMotor(_bin1, _bin2, 1, speed, _output[4]);
    _setMotor(_cin1, _cin2, 2, speed, -_output[4]);
    _setMotor(_din1, _din2, 3, -speed, -_output[4]);
    _enableDrivers();
    return MotionResult::Running;
}

void Run::setAim(int Aim) {
    aim = Aim;
}

void Run::syncAimToCurrentHeading() {
    aim = static_cast<int>(std::lround(_input[4]));
    _setpoint[4] = static_cast<double>(aim);
}

void Run::Getdata() {
    Serial.printf("%.2f %.2f %.2f %.2f %.2f    ",
                  _input[0], _input[1], _input[2], _input[3], _input[4]);
    Serial.printf("%.2f %.2f %.2f %.2f %.2f\n",
                  _output[0], _output[1], _output[2], _output[3], _output[4]);
}

int Run::getAim() {
    return aim;
}

bool Run::imuReady() const {
    return _imuReady;
}
