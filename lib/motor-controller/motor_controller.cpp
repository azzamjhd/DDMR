#include "motor_controller.hpp"

const float FORWARD_VELOCITY = 0.5;
const float BACKWARD_VELOCITY = -0.5;
const float TURN_VELOCITY = 3;
const float VELOCITY_SCALING_FACTOR = 0.065;

MotorController::MotorController(MotorDriver* rightMotor,
                                 MotorDriver* leftMotor,
                                 float distBetweenWheels)
    : _rightMotor(rightMotor),
      _leftMotor(leftMotor),
      _motorUpdateTimer(1000 / MOTOR_RUN_FREQ),
      _dist_between_wheels(distBetweenWheels) {
  _pose = {0, 0, 0};
  _cmdVel = {0, 0};
}

void MotorController::setCmdVel(CmdVel cmdVel) {
  _cmdVel = cmdVel;
  _computeWheelSpeeds();
}

void MotorController::getCmdVel(CmdVel& cmdVel) const {
  cmdVel = _cmdVel;
}

void MotorController::getPose(Pose& pose) const {
  pose = _pose;
}

void MotorController::reset() {
  _rightMotor->reset();
  _leftMotor->reset();
  _pose = {0, 0, 0};
  _cmdVel = {0, 0};
}

void MotorController::run() {
  _computePose();
  _rightMotor->run();
  _leftMotor->run();
}

void MotorController::calibrate() {
  _computePose();
  _rightMotor->calibrate();
  _leftMotor->calibrate();
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
  _pose.theta = atan2(sin(_pose.theta), cos(_pose.theta));
  // if (_pose.theta > PI) _pose.theta -= 2 * PI;
  // if (_pose.theta < -PI) _pose.theta += 2 * PI;

  _prev_right_dist = rightMotorData.distance;
  _prev_left_dist = leftMotorData.distance;
}

void MotorController::printPose() const {
  Serial.print("Pose: ");
  Serial.print(_pose.x);
  Serial.print(", ");
  Serial.print(_pose.y);
  Serial.print(", ");
  Serial.println(_pose.theta * 180 / PI);
}

void MotorController::moveOpenLoop(int leftPWM, int rightPWM) {
  _rightMotor->setPWM(rightPWM);
  _leftMotor->setPWM(leftPWM);
}

void MotorController::getMotorData(MotorData& rightMotorData,
                                   MotorData& leftMotorData) const {
  _rightMotor->getMotorData(rightMotorData);
  _leftMotor->getMotorData(leftMotorData);
}

void MotorController::setPIDGains(PIDGains pidGains) {
  _rightMotor->setPIDGains(pidGains);
  _leftMotor->setPIDGains(pidGains);
}

PIDGains MotorController::getPIDGains() const {
  return _rightMotor->getPIDGains();
}

void MotorController::setMaxVelocity(float maxVelocity) {
  _rightMotor->setMaxVelocity(maxVelocity);
  _leftMotor->setMaxVelocity(maxVelocity);
}

void MotorController::setMotorFreq(int freq) {
  MOTOR_RUN_FREQ = freq;
}

uint8_t MotorController::calculateChecksum(const uint8_t* buffer,
                                           size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; i++) {
    checksum ^= buffer[i];
  }
  return checksum;
}

template <typename T>
void MotorController::sendStructData(const T& data, DataType type) {
  const uint8_t* dataptr = reinterpret_cast<const uint8_t*>(&data);
  // Start byte
  Serial.write(0xAA);
  // Send Data
  Serial.write(type);
  Serial.write(dataptr, sizeof(T));
  // Checksum
  uint8_t checksum = calculateChecksum(dataptr, sizeof(T));
  Serial.write(checksum);
  // End byte
  Serial.write(0x55);
}

void MotorController::printStructData(const RobotData& data) {
  Serial.print("Pose: ");
  Serial.print(data.pose.x);
  Serial.print(", ");
  Serial.print(data.pose.y);
  Serial.print(", ");
  Serial.print(data.pose.theta);
  Serial.print("\tUltrasonic: ");
  Serial.print(data.ultrasonic[0]);
  Serial.print(", ");
  Serial.print(data.ultrasonic[1]);
  Serial.print(", ");
  Serial.print(data.ultrasonic[2]);
  Serial.print("\tSpeed: ");
  Serial.print(data.speed[0]);
  Serial.print(", ");
  Serial.println(data.speed[1]);
}

void MotorController::sendPidGains() {
  PIDGains pidGains = getPIDGains();
  sendStructData(pidGains, DataType::PID_GAINS);
}

void MotorController::sendStatus() {
  RobotData robotData;
  robotData.pose = _pose;
  robotData.ultrasonic[0] = _front_distance;
  robotData.ultrasonic[1] = _left_distance;
  robotData.ultrasonic[2] = _right_distance;
  MotorData rightMotorData, leftMotorData;
  getMotorData(rightMotorData, leftMotorData);
  robotData.speed[0] = leftMotorData.velocity;
  robotData.speed[1] = rightMotorData.velocity;

  // sendStructData(robotData, DataType::ROBOT_STATUS);
  printStructData(robotData);
}

void MotorController::SerialScan() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    if (input.length() > 0) {
      int x, w;
      int Kp_f, Ki_f, Kd_f;
      int x_f, y_f, theta_f;
      CmdVel cmdVel;
      Pose goalPose;
      PIDGains pidGains;

      if (sscanf(input.c_str(), "c %d %d", &x, &w) == 2) {
        cmdVel.x = float(x) / 1000.0;
        cmdVel.w = float(w) / 1000.0;
        setCmdVel(_cmdVel);
      } else if (sscanf(input.c_str(), "p %d %d %d", &Kp_f, &Ki_f, &Kd_f) ==
                 3) {
        pidGains.Kp = float(Kp_f) / 1000.0;
        pidGains.Ki = float(Ki_f) / 1000.0;
        pidGains.Kd = float(Kd_f) / 1000.0;
        setPIDGains(pidGains);
      } else if (input == "g") {
        sendPidGains();
      } else if (input == "s") {
        sendStatus();
      } else if (input == "r") {
        reset();
      } else {
        Serial.println("Invalid input format.");
      }
    }
  }
}

void MotorController::setFrontDistance(int frontDistance) {
  _front_distance = frontDistance;
}

int MotorController::getFrontDistance() const {
  return _front_distance;
}

void MotorController::setLeftDistance(int leftDistance) {
  _left_distance = leftDistance;
}

int MotorController::getLeftDistance() const {
  return _left_distance;
}

void MotorController::setRightDistance(int rightDistance) {
  _right_distance = rightDistance;
}

int MotorController::getRightDistance() const {
  return _right_distance;
}

CmdVel MotorController::contraintAutoCmd(CmdVel cmdVel, float x, float w) {
  cmdVel.x = constrain(cmdVel.x, -x, x);
  cmdVel.w = constrain(cmdVel.w, -w, w);
  return cmdVel;
}

void MotorController::moveByState(State currentState) {
  CmdVel currentCmdVel;

  switch (currentState) {
    case MOVE_TO_GOAL:
      currentCmdVel.x = _k_rho * _rho;
      currentCmdVel.w = _k_alpha * _alpha;
      currentCmdVel = contraintAutoCmd(currentCmdVel, _max_linear_velocity,
                                       _max_angular_velocity);
      setCmdVel(currentCmdVel);
      break;
    case ADJUST_ANGLE:
      if (_use_beta) {
        currentCmdVel.x = 0;
        currentCmdVel.w = _k_beta * _beta;
      } else {
        currentCmdVel.x = 0;
        currentCmdVel.w = 0;
      }
      currentCmdVel = contraintAutoCmd(currentCmdVel, _max_linear_velocity,
                                       _max_angular_velocity);
      setCmdVel(currentCmdVel);
      break;
    case AVOID_OBSTACLE:
      bool front_obstacle = false;
      bool left_obstacle = false;
      bool right_obstacle = false;

      if (_front_distance < _obstacle_threshold)
        front_obstacle = true;
      if (_left_distance < _obstacle_threshold)
        left_obstacle = true;
      if (_right_distance < _obstacle_threshold)
        right_obstacle = true;

      if (front_obstacle || right_obstacle) {
        currentCmdVel = {0, _max_angular_velocity};
      } else if (left_obstacle) {
        currentCmdVel = {0, -_max_angular_velocity};
      } else if (front_obstacle && left_obstacle) {
        currentCmdVel = {0, _max_angular_velocity};
      } else {
        currentCmdVel = {0};
      }

      currentCmdVel = contraintAutoCmd(currentCmdVel, _max_linear_velocity,
                                       _max_angular_velocity);
      setCmdVel(currentCmdVel);
      break;
  }
}