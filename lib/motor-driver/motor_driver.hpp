#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <Arduino.h>
#include <PID_v1.h>
#include "encoder.hpp"

typedef struct {
    double Kp;
    double Ki;
    double Kd;
} PIDGains;

typedef struct {
    float rpm;
    float angularVelocity;
    float velocity;
    float distance;
    float angle;
} MotorData;

enum class MotorDirection { CW, CCW, STOP };
enum class MotorMode { OPEN_LOOP, CLOSED_LOOP };

class MotorDriver {
   private:
    uint8_t _pinEn;
    uint8_t _pinIn1;
    uint8_t _pinIn2;

    float _wheelRadius;
    int _countPerRev;
    bool _reverse;
    float _MAX_VELOCITY = 1.0; // m/s

    MotorData _motorData;
    int _lastEncoderReading;
    unsigned long _lastDataReadingTime;

    double _setpoint, _input, _output;
    PIDGains _pidGains;
    PID _pid = PID(&_input, &_output, &_setpoint, 0, 0, 0, DIRECT);

    MotorDirection _motorDir;
    MotorMode _motorMode;
    Encoder *_encoder;
    uint8_t _pwm;
   public:
    MotorDriver(uint8_t pinEn, uint8_t pinIn1, uint8_t pinIn2, bool reverse = false);
    MotorDriver(uint8_t pinEn, uint8_t pinIn1, uint8_t pinIn2, Encoder *encoder, float wheelRadius, int countPerRev, bool reverse = false);
    
    void run(void);
    void printStatus(void);
    void reset(void);

    // Set
    void setVelocity(float velocity);
    void setPWM(int pwm, MotorMode mode = MotorMode::OPEN_LOOP);
    void setMode(MotorMode mode);
    void setPIDGains(PIDGains &pidGains);

    // Get
    void getMotorData(MotorData &motorData);
    float getWheelRadius(void);
    int getCountPerRev(void);
    PIDGains getPIDGains(void);

   private:
    void _sendPWM(void);
    void _setDirection(MotorDirection dir);
    void _initPins(void);
    float _computeRPM(void);
    float _computeAngularVelocity(float rpm);
    float _computeVelocity(float angularVelocity);
    float _computeDistance(void);
    float _computeWheelAngle(void);
    void _computeMotorData(void);
};

#endif // MOTOR_DRIVER_HPP