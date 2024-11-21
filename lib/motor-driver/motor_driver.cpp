#include "motor_driver.hpp"

MotorDriver::MotorDriver(uint8_t pinEn,
                         uint8_t pinIn1,
                         uint8_t pinIn2,
                         bool reverse)
    : _pinEn(pinEn), _pinIn1(pinIn1), _pinIn2(pinIn2), _reverse(reverse) {
  _pid.SetMode(AUTOMATIC);
  _pid.SetOutputLimits(-255, 255);
  _initPins();
  _motorMode = MotorMode::OPEN_LOOP;
}

MotorDriver::MotorDriver(uint8_t pinEn,
                         uint8_t pinIn1,
                         uint8_t pinIn2,
                         Encoder* encoder,
                         float wheelRadius,
                         int countPerRev,
                         bool reverse)
    : _pinEn(pinEn),
      _pinIn1(pinIn1),
      _pinIn2(pinIn2),
      _wheelRadius(wheelRadius),
      _countPerRev(countPerRev),
      _reverse(reverse),
      _encoder(encoder) {
  _pid.SetMode(AUTOMATIC);
  _pid.SetOutputLimits(-255, 255);
  _initPins();
  _motorMode = MotorMode::CLOSED_LOOP;
  _encoder->reset();
  _lastEncoderReading = encoder->getCount();
  _lastDataReadingTime = millis();
}

void MotorDriver::_initPins() {
  pinMode(_pinEn, OUTPUT);
  pinMode(_pinIn1, OUTPUT);
  pinMode(_pinIn2, OUTPUT);
}

void MotorDriver::reset() {
  if (_encoder == nullptr) {
    return;
  }
  _encoder->reset();
  _lastEncoderReading = _encoder->getCount();
  _lastDataReadingTime = millis();
  setPWM(0);
}

void MotorDriver::setPWM(int pwm, MotorMode mode) {
  setMode(mode);
  pwm = constrain(pwm, -255, 255);

  if (pwm < 0) {
    _setDirection(MotorDirection::CCW);
    _pwm = -pwm;
  } else if (pwm > 0) {
    _setDirection(MotorDirection::CW);
    _pwm = pwm;
  } else {
    _setDirection(MotorDirection::STOP);
    _pwm = 0;
  }
}

void MotorDriver::_setDirection(MotorDirection dir) {
  bool in1State = LOW;
  bool in2State = LOW;

  switch (dir) {
    case MotorDirection::CW:
      in1State = _reverse ? HIGH : LOW;
      in2State = _reverse ? LOW : HIGH;
      break;
    case MotorDirection::CCW:
      in1State = _reverse ? LOW : HIGH;
      in2State = _reverse ? HIGH : LOW;
      break;
    case MotorDirection::STOP:
      break;
  }

  digitalWrite(_pinIn1, in1State);
  digitalWrite(_pinIn2, in2State);
}

void MotorDriver::setMode(MotorMode mode) {
  if (_encoder == nullptr) {
    _motorMode = MotorMode::OPEN_LOOP;
    return;
  }
  _motorMode = mode;
}

void MotorDriver::setVelocity(float velocity) {
  velocity = constrain(velocity, -_MAX_VELOCITY, _MAX_VELOCITY);
  if (_encoder == nullptr) {
    return;
  }
  _motorMode = MotorMode::CLOSED_LOOP;
  _setpoint = velocity;
}

void MotorDriver::setMaxVelocity(float maxVelocity) {
  _MAX_VELOCITY = maxVelocity;
}

void MotorDriver::getMotorData(MotorData& motorData) {
  motorData = _motorData;
}

float MotorDriver::getWheelRadius() {
  return _wheelRadius;
}

int MotorDriver::getCountPerRev() {
  return _countPerRev;
}

void MotorDriver::setPIDGains(PIDGains& pidGains) {
  _pid.SetTunings(pidGains.Kp, pidGains.Ki, pidGains.Kd);
}

PIDGains MotorDriver::getPIDGains() {
  PIDGains pidGains = {_pid.GetKp(), _pid.GetKi(), _pid.GetKd()};
  return pidGains;
}

void MotorDriver::_sendPWM() {
  analogWrite(_pinEn, _pwm);
}

void MotorDriver::run() {
  if (_motorMode == MotorMode::CLOSED_LOOP) {
    _computeMotorData();
    _input = _motorData.velocity;
    _pid.Compute();
    setPWM(_output, MotorMode::CLOSED_LOOP);
  }
  // Serial.print("SP:");Serial.print(_setpoint);Serial.print(",");
  _sendPWM();
}

void MotorDriver::calibrate() {
  if (_motorMode == MotorMode::CLOSED_LOOP) {
    _computeMotorData();
    _input = _motorData.velocity;
    _pid.Compute();
    // setPWM(_output, MotorMode::CLOSED_LOOP);
  }
  // Serial.print("SP:");Serial.print(_setpoint);Serial.print(",");
  // _sendPWM();
}

void MotorDriver::printStatus() {
  // Serial.print("w: ");
  Serial.print(_motorData.angularVelocity);
  Serial.print("\t");
  // Serial.print(" rad/s | v: ");
  Serial.print(_motorData.velocity);
  Serial.print("\t");
  // Serial.print(" m/s | d: ");
  Serial.print(_motorData.distance);
  Serial.print("\t");
  // Serial.print(" m | angel: ");
  Serial.print(_motorData.angle);
  Serial.print("\t");
  // Serial.println(" rad | pwm:");
  Serial.print(_pwm);
}

float MotorDriver::_computeRPM() {
  auto count = _encoder->getCount();
  auto currentTime = millis();
  auto dt = currentTime - _lastDataReadingTime;
  auto dCount = count - _lastEncoderReading;
  float rpm = (float(dCount) / dt) * 60000.0 / _countPerRev;
  _lastEncoderReading = count;
  _lastDataReadingTime = currentTime;
  return rpm;
}

float MotorDriver::_computeAngularVelocity(float rpm) {
  return rpm * 2 * PI / 60;
}

float MotorDriver::_computeVelocity(float angular_velocity) {
  return angular_velocity * _wheelRadius;
}

float MotorDriver::_computeDistance() {
  int count = _encoder->getCount();
  return count * 2 * PI * _wheelRadius / _countPerRev;
}

float MotorDriver::_computeWheelAngle() {
  int count = _encoder->getCount();
  return count * 2 * PI / _countPerRev;
}

void MotorDriver::_computeMotorData() {
  float rpm = _computeRPM();
  _motorData.angularVelocity = _computeAngularVelocity(rpm);
  _motorData.velocity = _computeVelocity(_motorData.angularVelocity);
  _motorData.distance = _computeDistance();
  _motorData.angle = _computeWheelAngle();
}