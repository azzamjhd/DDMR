#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <Arduino.h>
#include "motor_driver.hpp"
#include "timer_api.hpp"

enum State { MOVE_TO_GOAL, ADJUST_ANGLE, AVOID_OBSTACLE };

typedef struct {
  float x;
  float y;
  float theta;
} Pose;

typedef struct {
  float x;
  float w;
} CmdVel;

enum DataType : uint8_t {
  PID_GAINS = 'p',
  ROBOT_STATUS = 's',
};

struct RobotData {
  Pose pose;
  uint16_t ultrasonic[3];
  float speed[2];
} __attribute__((packed));

class MotorController {
 private:
  MotorDriver* _rightMotor;
  MotorDriver* _leftMotor;
  TimerAPI _motorUpdateTimer;

  RobotData _robotData;
  Pose _pose;
  CmdVel _cmdVel;
  float _dist_between_wheels;
  float _prev_right_dist;
  float _prev_left_dist;

  // Auto mode variables
  bool AUTO_MODE = false;
  float _max_linear_velocity = 0.2;
  float _max_angular_velocity = 2.0;
  int _front_distance;
  int _left_distance;
  int _right_distance;
  bool _use_beta = false;
  float _rho, _alpha, _beta;
  float _k_rho = 0.5, _k_alpha = 2.0, _k_beta = 1.0;
  int _obstacle_threshold = 20;  // in centimeters
  float _goal_tolerance = 0.05;  // in meters

  int MOTOR_RUN_FREQ = 10;  // in Hz

  /// @brief Compute the pose of the robot using odometry data
  void _computePose();
  /// @brief Compute the wheel speeds of the robot using the cmd_vel data for
  /// each motor
  void _computeWheelSpeeds();
  uint8_t calculateChecksum(const uint8_t* buffer, size_t length);
  template <typename T>
  void sendStructData(const T& data, DataType type);
  void printStructData(const RobotData& data);

  CmdVel contraintAutoCmd(CmdVel cmdVel, float x, float w);
  void moveByState(State currentState);

 public:
  /**
   * @brief Construct a new Motor Controller object
   *
   * @param rightMotor Right motor driver
   * @param leftMotor Left motor driver
   * @param distBetweenWheels Distance between the wheels in meters
   */
  MotorController(MotorDriver* rightMotor,
                  MotorDriver* leftMotor,
                  float distBetweenWheels);
  /**
   * @brief Get the Pose object
   * @param pose Pose object. pose = {x, y, theta}. x is the x-coordinate in
   * meters, y is the y-coordinate in meters, and theta is the orientation in
   * radians
   */
  void getPose(Pose& pose) const;
  /**
   * @brief print the pose of the robot without the motor running
   */
  void calibrate();
  /**
   * @brief Set the Cmd Vel object
   * @param cmdVel Cmd Vel object = {x, w}. x is the linear velocity in m/s and
   * w is the angular velocity in rad/s
   */
  void setCmdVel(CmdVel cmdVel);
  /// @brief Get current cmd_vel
  /// @param cmdVel
  void getCmdVel(CmdVel& cmdVel) const;
  /// @brief Reset the motor controller
  void reset();
  /// @brief Run the motor controller
  void run();
  /// @brief Print the pose of the robot
  void printPose(void) const;
  /// @brief Move the robot forward at FORWARD_VELOCITY m/s
  void moveForward();
  /// @brief Move the robot backward at BACKWARD_VELOCITY m/s
  void moveBackward();
  /// @brief Turn the robot left at TURN_VELOCITY rad/s
  void turnLeft();
  /// @brief Turn the robot right at TURN_VELOCITY rad/s
  void turnRight();
  /// @brief Stop the robot
  void stop();
  /**
   * @brief Move the robot using open loop control
   *
   * @param leftPWM PWM value for the left motor. Range: -255 to 255
   * @param rightPWM PWM value for the right motor. Range: -255 to 255
   */
  void moveOpenLoop(int leftPWM, int rightPWM);
  /**
   * @brief Get the Motor Data object
   *
   * @param rightMotorData
   * @param leftMotorData
   */
  void getMotorData(MotorData& rightMotorData, MotorData& leftMotorData) const;
  /// @brief Reset the pose of the robot
  void resetPose();
  /**
   * @brief Set the PID gains of the motors
   *
   * @param pidGains PID gains. pidGains = {Kp, Ki, Kd}
   */
  void setPIDGains(PIDGains pidGains);
  /**
   * @brief Get the PID gains of the motors
   *
   * @return PIDGains = {Kp, Ki, Kd}
   */
  PIDGains getPIDGains(void) const;
  /**
   * @brief Set the Max Velocity object
   *
   * @param maxVelocity Maximum velocity in m/s
   */
  void setMaxVelocity(float maxVelocity);
  /**
   * @brief Set the Motor Freq object
   *
   * @param freq Frequency in Hz
   */
  void setMotorFreq(int freq);
  void sendPidGains();
  void sendStatus();
  void SerialScan();
  void setFrontDistance(int frontDistance);
  void setLeftDistance(int leftDistance);
  void setRightDistance(int rightDistance);
  int getFrontDistance() const;
  int getLeftDistance() const;
  int getRightDistance() const;
};

#endif  // MOTOR_CONTROLLER_HPP