#ifndef RUNNING_H
#define RUNNING_H

#include <Adafruit_MPU6050.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>

enum class MotionResult : uint8_t {
    Running,
    Completed,
    ImuFault
};

class Run {
  private:
    int _ain1, _ain2, _bin1, _bin2, _cin1, _cin2, _din1, _din2;
    int _pwma, _pwmb, _pwmc, _pwmd, _stbyab, _stbycd;
    double _output[5] = {0, 0, 0, 0, 0};
    double _input[5] = {0, 0, 0, 0, 0};
    double _setpoint[5] = {0, 0, 0, 0, 0};
    PID pid[5] = {
        PID(&_input[0], &_output[0], &_setpoint[0], 60, 4.4, 1.6, DIRECT),
        PID(&_input[1], &_output[1], &_setpoint[1], 60, 4.4, 1.6, DIRECT),
        PID(&_input[2], &_output[2], &_setpoint[2], 60, 4.4, 1.6, DIRECT),
        PID(&_input[3], &_output[3], &_setpoint[3], 60, 4.4, 1.6, DIRECT),
        PID(&_input[4], &_output[4], &_setpoint[4], 0.2, 0, 0, DIRECT)
    };
    sensors_event_t a, g, t;
    Adafruit_MPU6050 MPU;
    ESP32Encoder enc[4];
    bool _turnActive = false;
    int _turnDirection = 0;
    bool _imuReady = false;
    double Bias = 0;
    int aim = 0;

    void _clearMotorOutputs();
    void _disableDrivers();
    void _enableDrivers();
    void _resetPidState();
    void _setMotor(int pin1, int pin2, int motorIndex, int expect,
                   double correction);
    bool _updateHeading(int angle, float elapsedSeconds);
    MotionResult _rotate(int direction, int speed, float elapsedSeconds);
    MotionResult _turn(int direction, int speed, int angle,
                       float elapsedSeconds);

  public:
    Run(int ain1, int ain2, int bin1, int bin2, int cin1, int cin2,
        int din1, int din2, int pwma, int pwmb, int pwmc, int pwmd,
        int stbyab, int stbycd);
    void BeginSafe();
    void PIDSetup(double Kp = 60, double Ki = 4.4, double Kd = 1.6);
    bool MPUSetup(int SCL, int SDA);
    void EncoderSetup(int A_A, int A_B, int B_A, int B_B,
                      int C_A, int C_B, int D_A, int D_B);
    void Stop();
    MotionResult Forward(int speed, float elapsedSeconds);
    MotionResult Backward(int speed, float elapsedSeconds);
    // Continuous manual rotation. Unlike LeftTurn/RightTurn, these methods
    // have no target angle and run until the caller stops refreshing MOVE.
    MotionResult RotateLeft(int speed, float elapsedSeconds);
    MotionResult RotateRight(int speed, float elapsedSeconds);
    MotionResult LeftTurn(int speed, int angle, float elapsedSeconds);
    MotionResult RightTurn(int speed, int angle, float elapsedSeconds);
    MotionResult LeftShift(int speed, float elapsedSeconds);
    MotionResult RightShift(int speed, float elapsedSeconds);
    void setAim(int Aim);
    void syncAimToCurrentHeading();
    void Getdata();
    int getAim();
    bool imuReady() const;
};

#endif
