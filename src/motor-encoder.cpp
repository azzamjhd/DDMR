#include "base.h"
#include <Arduino.h>

// ===============================================
// =========== Motor and Encoder Class ===============
// ===============================================

MotorEncoder::MotorEncoder(int pwm, int in1, int in2, int pinA, int pinB, float wheelDiameter, int countPerRev, bool reverse) {
    _pwm = pwm;
    _in1 = in1;
    _in2 = in2;
    _pinA = pinA;
    _pinB = pinB;
    _wheelDiameter = wheelDiameter;
    _countPerRev = countPerRev;
    _count = 0;
    _curretState = 0;
    _lastState = 0;
    _reverse = reverse;

    pinMode(_pwm, OUTPUT);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_pinA, INPUT);
    pinMode(_pinB, INPUT);

    _pid.SetMode(AUTOMATIC);
    _pid.SetOutputLimits(-255, 255);
}

void MotorEncoder::run() {
    computeMotorData();
    _input = _motor_data.velocity;
    _pid.Compute();
    setSpeed(_output);
}

void MotorEncoder::reset() {
    _count = 0;
    _lastCount = 0;
    _lastTime = millis();
    _setpoint = 0;
    _input = 0;
    _output = 0;
    _motor_data.rpm = 0;
    _motor_data.angular_velocity = 0;
    _motor_data.velocity = 0;
    _motor_data.distance = 0;
}

void MotorEncoder::updateGains(double Kp, double Ki, double Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _pid.SetTunings(_Kp, _Ki, _Kd);
}

void MotorEncoder::getGains(double &Kp, double &Ki, double &Kd) {
    Kp = _pid.GetKp();
    Ki = _pid.GetKi();
    Kd = _pid.GetKd();
}

float MotorEncoder::getWheelDiameter() {
    return _wheelDiameter;
}

int MotorEncoder::getCountPerRev() {
    return _countPerRev;
}

void MotorEncoder::setMaxVelocity(float max_velocity) {
    _MAX_VELOCITY = max_velocity;
}

void MotorEncoder::setDir(MotorDirection dir) {
    switch (dir) {
        case MotorDirection::FORWARD:
            digitalWrite(_in1, LOW);
            digitalWrite(_in2, HIGH);
            break;
        case MotorDirection::BACKWARD:
            digitalWrite(_in1, HIGH);
            digitalWrite(_in2, LOW);
            break;
        case MotorDirection::STOP:
            digitalWrite(_in1, LOW);
            digitalWrite(_in2, LOW);
            break;
    }
}

void MotorEncoder::set_velocity(float velocity) {
    // velocity = _reverse ? -velocity : velocity;
    velocity = constrain(velocity, -_MAX_VELOCITY, _MAX_VELOCITY);
    _setpoint = velocity;
}

void MotorEncoder::setSpeed(int speed) {
    // speed = _reverse ? -speed : speed;
    if (speed > 0) {
        setDir(MotorDirection::FORWARD);
        analogWrite(_pwm, speed);
    } else if (speed < 0) {
        setDir(MotorDirection::FORWARD);
        analogWrite(_pwm, -speed);
    } else {
        setDir(MotorDirection::STOP);
        analogWrite(_pwm, 0);
    }
}

void MotorEncoder::update() {
    _curretState = digitalRead(_pinA) << 2 | digitalRead(_pinB);
    if (_curretState == 0b100 && _lastState == 0b01) {
        _count++;
    } else if (_curretState == 0b101 && _lastState == 0b00) {
        _count--;
    }
    _lastState = _curretState;
}

int MotorEncoder::getCount() {
    return _reverse ? _count * -1 : _count;
}

float MotorEncoder::_computeRPM() {
    auto count = getCount();
    auto currentTime = millis();
    auto dt_time = currentTime - _lastTime;
    int dt_count = count - _lastCount;
    float rpm = (float(dt_count/dt_time)) * 60000.0 / _countPerRev;
    _lastCount = count;
    _lastTime = currentTime;
    return rpm;
}

float MotorEncoder::_computeAngularVelocity(float rpm) {
    return rpm * 2 * PI / 60;
}

float MotorEncoder::_computeVelocity(float angular_velocity) {
    return angular_velocity * _wheelDiameter / 2;
}

float MotorEncoder::_computeDistance() {
    auto count = getCount();
    return count * 2 * PI * _wheelDiameter / 2 / _countPerRev;
}

float MotorEncoder::_computeAngle() {
    auto count = getCount();
    return count * 2 * PI / _countPerRev;
}

void MotorEncoder::computeMotorData(void) {
    _motor_data.rpm = _computeRPM();
    _motor_data.angular_velocity = _computeAngularVelocity(_motor_data.rpm);
    _motor_data.velocity = _computeVelocity(_motor_data.angular_velocity);
    _motor_data.distance = _computeDistance();
    _motor_data.angle = _computeAngle();
}

void MotorEncoder::getMotorData(MotorData &motor_data) {
    motor_data = _motor_data;
}