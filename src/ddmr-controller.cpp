#include <base.h>

// ===============================================
// =========== DDMR Controller Class ========================
// ===============================================

DDMRobot::DDMRobot(MotorEncoder *rightMotor, MotorEncoder *leftMotor, float wheelBaseDistance) {
    _rightMotor = rightMotor;
    _leftMotor = leftMotor;
    _wheelBaseDistance = wheelBaseDistance;
    _motorUpdateTimer.set_interval(1000/_MOTOR_RUN_FREQ); // hz to ms
    _pose = {0.0, 0.0, 0.0};
    _cmdVel = {0.0, 0.0};
    _prevRightDistance = 0;
    _prevLeftDistance = 0;
}

void DDMRobot::run() {
    if (_motorUpdateTimer.has_elapsed()) {
        _computePose();
        _rightMotor->run();
        _leftMotor->run();
    }
}

void DDMRobot::reset() {
    _leftMotor->reset();
    _rightMotor->reset();
    _pose = {0.0, 0.0, 0.0};
    _cmdVel = {0.0, 0.0};
    _prevRightDistance = 0;
    _prevLeftDistance = 0;
}

void DDMRobot::resetPose() {
    _pose = {0.0, 0.0, 0.0};
    _prevRightDistance = 0;
    _prevLeftDistance = 0;
    _rightMotor->reset();
    _leftMotor->reset();
}

void DDMRobot::_computeWheelSpeeds() {
    float V_R = (2 * _cmdVel.x + _cmdVel.w * _wheelBaseDistance) / _rightMotor->getWheelDiameter();
    float V_L = (2 * _cmdVel.x - _cmdVel.w * _wheelBaseDistance) / _leftMotor->getWheelDiameter();

    V_R = V_R * 0.0338;
    V_L = V_L * 0.0338;

    _rightMotor->set_velocity(V_R);
    _leftMotor->set_velocity(V_L);
}

void DDMRobot::_computePose() {
    MotorData rightMotorData, leftMotorData;
    _rightMotor->getMotorData(rightMotorData);
    _leftMotor->getMotorData(leftMotorData);

    float deltaRight = rightMotorData.distance - _prevRightDistance;
    float deltaLeft = leftMotorData.distance - _prevLeftDistance;
    float deltaCenter = (deltaRight + deltaLeft) / 2;
    float deltaTheta = (deltaRight - deltaLeft) / _wheelBaseDistance;

    _pose.x += deltaCenter * cos(_pose.theta);
    _pose.y += deltaCenter * sin(_pose.theta);
    _pose.theta += deltaTheta;
    if (_pose.theta > PI) _pose.theta -= 2*PI;
    if (_pose.theta < -PI) _pose.theta += 2*PI;

    _prevRightDistance = rightMotorData.distance;
    _prevLeftDistance = leftMotorData.distance;
}

void DDMRobot::move(float x, float w) {
    _cmdVel = {x, w};
    _computeWheelSpeeds();
}

void DDMRobot::moveOpenLoop(int speedR, int speedL) {
    _rightMotor->setSpeed(speedR);
    _leftMotor->setSpeed(speedL);
}

void DDMRobot::stop() {
    _cmdVel = {0.0, 0.0};
    _computeWheelSpeeds();
}

void DDMRobot::moveForward() {
    _cmdVel = {0.3, 0.0};
    _computeWheelSpeeds();
}

void DDMRobot::moveBackward() {
    _cmdVel = {-0.3, 0.0};
    _computeWheelSpeeds();
}

void DDMRobot::turnLeft() {
    _cmdVel = {0.0, 0.5};
    _computeWheelSpeeds();
}

void DDMRobot::turnRight() {
    _cmdVel = {0.0, -0.5};
    _computeWheelSpeeds();
}

void DDMRobot::getMotorData(MotorData &rightMotorData, MotorData &leftMotorData) {
    _rightMotor->getMotorData(rightMotorData);
    _leftMotor->getMotorData(leftMotorData);
}

void DDMRobot::updateMotorGains(double Kp, double Ki, double Kd) {
    _rightMotor->updateGains(Kp, Ki, Kd);
    _leftMotor->updateGains(Kp, Ki, Kd);
}

void DDMRobot::getMotorGains(double &Kp, double &Ki, double &Kd) {
    _rightMotor->getGains(Kp, Ki, Kd);
}

void DDMRobot::getPose(Pose &pose) {
    pose = _pose;
}