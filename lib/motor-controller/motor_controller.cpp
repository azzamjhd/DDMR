#include "motor_controller.hpp"

const float FORWARD_VELOCITY = 0.5;
const float BACKWARD_VELOCITY = -0.5;
const float TURN_VELOCITY = 3;
const float VELOCITY_SCALING_FACTOR = 0.065;

MotorController::MotorController(MotorDriver *rightMotor, MotorDriver *leftMotor, float distBetweenWheels)
    : 
    _rightMotor(rightMotor), 
    _leftMotor(leftMotor), 
    _motorUpdateTimer(1000 / MOTOR_RUN_FREQ),
    _dist_between_wheels(distBetweenWheels)
{
    _pose = {0, 0, 0};
    _cmdVel = {0, 0};
}

void MotorController::setCmdVel(CmdVel cmdVel) {
    _cmdVel = cmdVel;
    _computeWheelSpeeds();
}

void MotorController::getPose(Pose &pose) const { pose = _pose; }

void MotorController::reset() {
    _rightMotor->reset();
    _leftMotor->reset();
    _pose = {0, 0, 0};
    _cmdVel = {0, 0};
}

void MotorController::run() {
    if (_motorUpdateTimer.has_elapsed()) {
    }
        _computePose();
        _rightMotor->run();
        _leftMotor->run();
}

void MotorController::moveForward() {
    _cmdVel = {FORWARD_VELOCITY, 0};
    _computeWheelSpeeds();
}

void MotorController::moveBackward() {
    _cmdVel = {BACKWARD_VELOCITY, 0};
    _computeWheelSpeeds();
}

void MotorController::turnLeft() {
    _cmdVel = {0, TURN_VELOCITY};
    _computeWheelSpeeds();
}

void MotorController::turnRight() {
    _cmdVel = {0, -TURN_VELOCITY};
    _computeWheelSpeeds();
}

void MotorController::stop() {
    _cmdVel = {0, 0};
    _computeWheelSpeeds();
}

void MotorController::resetPose() {
    _pose = {0, 0, 0};
    _leftMotor->reset();
    _rightMotor->reset();
}

void MotorController::_computeWheelSpeeds() {
    float v_r = (2 * _cmdVel.x + _cmdVel.w * _dist_between_wheels) /
                (2 * _rightMotor->getWheelRadius());
    float v_l = (2 * _cmdVel.x - _cmdVel.w * _dist_between_wheels) /
                (2 * _leftMotor->getWheelRadius());

    v_r = v_r * VELOCITY_SCALING_FACTOR;
    v_l = v_l * VELOCITY_SCALING_FACTOR;

    // Serial.print("v_r: "); Serial.print(v_r); Serial.print("\t");
    // Serial.print("v_l: "); Serial.println(v_l);

    _rightMotor->setVelocity(v_r);
    _leftMotor->setVelocity(v_l);
}

void MotorController::_computePose() {
    MotorData rightMotorData, leftMotorData;
    _rightMotor->getMotorData(rightMotorData);
    _leftMotor->getMotorData(leftMotorData);

    float d_r = rightMotorData.distance - _prev_right_dist;
    float d_l = leftMotorData.distance - _prev_left_dist;
    float d_c = (d_r + d_l) / 2;
    float d_theta = (d_r - d_l) / _dist_between_wheels;

    _pose.x += d_c * cos(_pose.theta);
    _pose.y += d_c * sin(_pose.theta);
    _pose.theta += d_theta;
    if (_pose.theta > PI) _pose.theta -= 2 * PI;
    if (_pose.theta < -PI) _pose.theta += 2 * PI;

    _prev_right_dist = rightMotorData.distance;
    _prev_left_dist = leftMotorData.distance;
}

void MotorController::printPose() const {
    Serial.print("Pose: ");
    Serial.print(_pose.x);
    Serial.print(", ");
    Serial.print(_pose.y);
    Serial.print(", ");
    Serial.println(_pose.theta);
}

void MotorController::moveOpenLoop(int leftPWM, int rightPWM) {
    _rightMotor->setPWM(rightPWM);
    _leftMotor->setPWM(leftPWM);
}

void MotorController::getMotorData(MotorData &rightMotorData, MotorData &leftMotorData) const {
    _rightMotor->getMotorData(rightMotorData);
    _leftMotor->getMotorData(leftMotorData);
}

void MotorController::setPIDGains(PIDGains pidGains) {
    _rightMotor->setPIDGains(pidGains);
    _leftMotor->setPIDGains(pidGains);
}

PIDGains MotorController::getPIDGains() const { return _rightMotor->getPIDGains(); }

void MotorController::setMaxVelocity(float maxVelocity) {
    _rightMotor->setMaxVelocity(maxVelocity);
    _leftMotor->setMaxVelocity(maxVelocity);
}

void MotorController::setMotorFreq(int freq) { MOTOR_RUN_FREQ = freq; }