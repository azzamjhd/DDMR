#include <ArduinoJson.h>
#include <PID_v1.h>
#include <Ultrasonic.h>
#include <math.h>

#include "config.h"
#include "motor_controller.hpp"
#include "motor_driver.hpp"

#define DEBUG

// #define TEST_DRIVER
// #define TEST_CONTROLLER
#define TEST_CARTESIAN_CONTROL

#define USE_OBSTACLE_AVOIDANCE

#define WHELL_DIAMETER 0.065
#define COUNT_PER_REV 320
#define DIST_BETWEEN_WHEELS 0.20

double Kp = 20, Ki = 400, Kd = 0;
PIDGains pidGains = {Kp, Ki, Kd};
MotorData rightMotorData, leftMotorData;

Ultrasonic ultrasonic1(TRIG_PIN_1, ECHO_PIN_1);
Ultrasonic ultrasonic2(TRIG_PIN_2, ECHO_PIN_2);
Ultrasonic ultrasonic3(TRIG_PIN_3, ECHO_PIN_3);

Encoder rightEncoder(R_ENC_A, R_ENC_B);
Encoder leftEncoder(L_ENC_A, L_ENC_B, true);

MotorDriver rightMotor(PWM_A,
                       A_IN_1,
                       A_IN_2,
                       &rightEncoder,
                       WHELL_DIAMETER / 2,
                       COUNT_PER_REV);
MotorDriver leftMotor(PWM_B,
                      B_IN_1,
                      B_IN_2,
                      &leftEncoder,
                      WHELL_DIAMETER / 2,
                      COUNT_PER_REV);

MotorController robot(&rightMotor, &leftMotor, DIST_BETWEEN_WHEELS);

State state = MOVE_TO_GOAL;

CmdVel cmdVel;
Pose currentPose;
RobotData robotData, lastRobotData;
Pose goalPose = {0, 0, NAN};

bool AUTOMODE = false;

// Odometry controls
float MAX_LINEAR_VELOCITY = 0.1;
float MAX_ANGULAR_VELOCITY = 0.5;
int WALL_FOLLOW_DISTANCE = 10;
int DISTANCE_THRESHOLD = 20;

double I_dist = 0, O_dist = 0;
double Kp_pos = 1.5, Ki_pos = 0.01, Kd_pos = 0.5;
PID distancePID(&I_dist, &O_dist, 0, Kp_pos, Ki_pos, Kd_pos, DIRECT);

double Kp_sensor = 4, Ki_sensor = 2, Kd_sensor = 0;

double I_left_sensor = 0, O_left_sensor = 0, I_right_sensor = 0,
       O_right_sensor = 0;
double SP_sensor = WALL_FOLLOW_DISTANCE;
PID leftSensorPID(&I_left_sensor,
                  &O_left_sensor,
                  &SP_sensor,
                  Kp_sensor,
                  Ki_sensor,
                  Kd_sensor,
                  DIRECT);
PID rightSensorPID(&I_right_sensor,
                   &O_right_sensor,
                   &SP_sensor,
                   Kp_sensor,
                   Ki_sensor,
                   Kd_sensor,
                   DIRECT);

double I_head = 0, O_head = 0, SP_head = 0;
double Kp_angle = 4, Ki_angle = 2, Kd_angle = 0;
PID headingPID(&I_head,
               &O_head,
               &SP_head,
               Kp_angle,
               Ki_angle,
               Kd_angle,
               DIRECT);

uint8_t calculateChecksum(const uint8_t* buffer, size_t length);
template <typename T>
void sendStructData(const T& data, DataType type);
void sendString();
void sendPidGains();
void sendStatus();
void SerialScan();

void rightEncoderISR() {
  rightEncoder.count_isr();
}
void leftEncoderISR() {
  leftEncoder.count_isr();
}
void setupInterrupts(void) {
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, RISING);
}

void setup() {
  Serial.begin(115200);
  setupInterrupts();
  robot.setPIDGains(pidGains);

  distancePID.SetMode(AUTOMATIC);
  distancePID.SetOutputLimits(-MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetOutputLimits(-MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  rightSensorPID.SetMode(AUTOMATIC);
  rightSensorPID.SetOutputLimits(-MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  leftSensorPID.SetMode(AUTOMATIC);
  leftSensorPID.SetOutputLimits(-MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  cmdVel = {0.1, 0};
}

float useDistancePID(float input) {
  I_dist = input;
  distancePID.Compute();
  return O_dist;
}

float useHeadingPID(float setpoint) {
  SP_head = setpoint;
  I_head = currentPose.theta;
  headingPID.Compute();
  return O_head;
}

void followWall();
void goToGoal(Pose currentPose, Pose goalPose);

// ====================== MAIN LOOP ======================

static unsigned long lastTime = 0;

void loop() {
#ifdef TEST_DRIVER
  robot.calibrate();

  rightMotor.getMotorData(rightMotorData);
  leftMotor.getMotorData(leftMotorData);

  int front_distance = ultrasonic1.read();
  int left_distance = ultrasonic2.read();
  int right_distance = ultrasonic3.read();

  Serial.print(front_distance);
  Serial.print("\t");
  Serial.print(left_distance);
  Serial.print("\t");
  Serial.print(right_distance);
  Serial.println();

#elif defined(TEST_CONTROLLER)
  // autoRun();

  robot.run();
  // serialCommand.readSerial();
  robot.printPose();
  rightMotor.getMotorData(rightMotorData);
  leftMotor.getMotorData(leftMotorData);

  if (Serial.available()) {
    String input =
        Serial.readStringUntil('\n');  // Read the input until newline character
    Serial.print("Input: ");
    Serial.println(input);  // Print the inputted data back

    if (input.length() > 0) {
      int x, w;
      if (sscanf(input.c_str(), "c %d %d", &x, &w) == 2) {
        cmdVel.x = float(x) / 1000.0;
        cmdVel.w = float(w) / 1000.0;
        robot.setCmdVel(cmdVel);
        // Serial.print("Set cmdVel to x: "); Serial.print(x); Serial.print(",
        // w: "); Serial.println(w);
      } else {
        Serial.println(
            "Invalid input format. Please enter two integers separated by a "
            "space.");
      }
    }
  }

#elif defined(TEST_CARTESIAN_CONTROL)
  robot.setFrontDistance(ultrasonic2.read());
  robot.setLeftDistance(ultrasonic3.read());
  robot.setRightDistance(ultrasonic1.read());

  rightMotor.getMotorData(rightMotorData);
  leftMotor.getMotorData(leftMotorData);
  robot.getPose(currentPose);

  SerialScan();

  if (AUTOMODE) {
    goToGoal(currentPose, goalPose);
  }

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 50) {
    lastTime = currentTime;
    sendStatus();
  }

  robot.run();
  // robot.printPose();
#endif
}

// ============== FUNCTION DEFINITIONS ==============

uint8_t calculateChecksum(const uint8_t* buffer, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; i++) {
    checksum ^= buffer[i];
  }
  return checksum;
}

template <typename T>
void sendStructData(const T& data, DataType type) {
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

void sendString() {
  Serial.print("Pose: ");
  Serial.print(robotData.pose.x);
  Serial.print(", ");
  Serial.print(robotData.pose.y);
  Serial.print(", ");
  Serial.print(robotData.pose.theta);
  Serial.print("\tUltrasonic: ");
  Serial.print(robotData.ultrasonic[0]);
  Serial.print(", ");
  Serial.print(robotData.ultrasonic[1]);
  Serial.print(", ");
  Serial.print(robotData.ultrasonic[2]);
  Serial.print("\tSpeed: ");
  Serial.print(robotData.speed[0]);
  Serial.print(", ");
  Serial.println(robotData.speed[1]);
}

void sendPidGains() {
  PIDGains pidGains = robot.getPIDGains();
  sendStructData(pidGains, DataType::PID_GAINS);
}

void sendStatus() {
  robotData.pose = currentPose;
  robotData.ultrasonic[0] = robot.getLeftDistance();
  robotData.ultrasonic[1] = robot.getFrontDistance();
  robotData.ultrasonic[2] = robot.getRightDistance();
  robotData.speed[0] = leftMotorData.velocity;
  robotData.speed[1] = rightMotorData.velocity;

  // sendStructData(robotData, DataType::ROBOT_STATUS);
  sendString();
}

void SerialScan() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input.length() > 0) {
      int x, w;
      int Kp_f, Ki_f, Kd_f;
      int x_f, y_f, theta_f;

      if (sscanf(input.c_str(), "c %d %d", &x, &w) == 2) {
        cmdVel.x = float(x) / 1000.0;
        cmdVel.w = float(w) / 1000.0;
        robot.setCmdVel(cmdVel);
        AUTOMODE = false;

      } else if (sscanf(input.c_str(), "p %d %d %d", &Kp_f, &Ki_f, &Kd_f) ==
                 3) {
        pidGains.Kp = float(Kp_f) / 1000.0;
        pidGains.Ki = float(Ki_f) / 1000.0;
        pidGains.Kd = float(Kd_f) / 1000.0;
        robot.setPIDGains(pidGains);

      } else if (sscanf(input.c_str(), "t %d %d", &x_f, &y_f) == 2) {
        goalPose.x = float(x_f) / 1000.0;
        goalPose.y = float(y_f) / 1000.0;
        goalPose.theta = NAN;
        AUTOMODE = true;

        // state = MOVE_TO_GOAL;
      } else if (input == "g") {
        sendPidGains();

      } else if (input == "s") {
        sendStatus();

      } else if (input == "r") {
        AUTOMODE = false;
        robot.reset();
        currentPose = {0};
        goalPose = {0, 0, NAN};
      } else {
        Serial.println(
            "Invalid input format. Please enter two integers separated by a "
            "space.");
      }
    }
  }
}

void followWall() {
  int front_distance = robot.getFrontDistance();
  int left_distance = robot.getLeftDistance();
  int right_distance = robot.getRightDistance();

  if (front_distance < DISTANCE_THRESHOLD) {
    cmdVel.x = 0;
    cmdVel.w = MAX_ANGULAR_VELOCITY;
  } else if (left_distance < WALL_FOLLOW_DISTANCE) {
    I_left_sensor = left_distance;
    leftSensorPID.Compute();
    cmdVel.x = 0;
    cmdVel.w = -O_left_sensor;
  } else if (right_distance < WALL_FOLLOW_DISTANCE) {
    I_right_sensor = right_distance;
    rightSensorPID.Compute();
    cmdVel.x = 0;
    cmdVel.w = O_right_sensor;
  } else {
    cmdVel.x = MAX_LINEAR_VELOCITY;
    cmdVel.w = 0;
  }

  robot.setCmdVel(cmdVel);
}

void goToGoal(Pose currentPose, Pose goalPose) {
  CmdVel currentCmdVel;

  bool use_beta = false;
  float k_rho = 0.5;
  float k_alpha = 2.0;
  float k_beta = 1.0;
  float goal_tolerance = 0.05;

  float alpha, beta;

  float dx = goalPose.x - currentPose.x;
  float dy = goalPose.y - currentPose.y;
  float rho = sqrt(dx * dx + dy * dy);
  float direction = atan2(dy, dx);
  alpha = direction - currentPose.theta;
  alpha = atan2(sin(alpha), cos(alpha));

  if (!isnan(goalPose.theta)) {
    beta = goalPose.theta - currentPose.theta;
    beta = atan2(sin(beta), cos(beta));
    use_beta = true;
  } else {
    beta = 0;
    use_beta = false;
  }

  // Check for obstacles
  int front_distance = robot.getFrontDistance();
  int left_distance = robot.getLeftDistance();
  int right_distance = robot.getRightDistance();
  int obstacle_threshold = 20;  // Threshold distance in cm
  bool front_obstacle = false, left_obstacle = false, right_obstacle = false;

#ifndef USE_OBSTACLE_AVOIDANCE
  if (rho <= goal_tolerance) {
    state = ADJUST_ANGLE;
  } else {
    state = MOVE_TO_GOAL;
  }
#elif defined(USE_OBSTACLE_AVOIDANCE)
  if (rho <= goal_tolerance) {
    state = ADJUST_ANGLE;
  } else if (front_distance < obstacle_threshold ||
             left_distance < obstacle_threshold ||
             right_distance < obstacle_threshold) {
    state = AVOID_OBSTACLE;
  } else {
    state = MOVE_TO_GOAL;
  }
#endif

  switch (state) {
    case MOVE_TO_GOAL:
      // currentCmdVel.x = -useDistancePID(rho);
      // currentCmdVel.w = useHeadingPID(direction);
      currentCmdVel.x = k_rho * rho;
      currentCmdVel.w = k_alpha * alpha;

      break;
    case ADJUST_ANGLE:
      if (use_beta) {
        currentCmdVel.x = 0;
        currentCmdVel.w = k_beta * beta;
      } else {
        currentCmdVel.x = 0;
        currentCmdVel.w = 0;
      }

      break;
    case AVOID_OBSTACLE:
      int front_distance = robot.getFrontDistance();
      int left_distance = robot.getLeftDistance();
      int right_distance = robot.getRightDistance();

      SP_sensor = DISTANCE_THRESHOLD;

      if (front_distance < DISTANCE_THRESHOLD) {
        currentCmdVel.x = 0;
        currentCmdVel.w = MAX_ANGULAR_VELOCITY;
      }
      if (left_distance < DISTANCE_THRESHOLD) {
        I_left_sensor = left_distance;
        leftSensorPID.Compute();
        currentCmdVel.x = 0;
        currentCmdVel.w = -O_left_sensor;
      } else if (right_distance < DISTANCE_THRESHOLD) {
        I_right_sensor = right_distance;
        rightSensorPID.Compute();
        currentCmdVel.x = 0;
        currentCmdVel.w = O_right_sensor;
      } else {
        currentCmdVel.x = MAX_LINEAR_VELOCITY;
        currentCmdVel.w = 0;
      }

      break;
  }

  currentCmdVel.x =
      constrain(currentCmdVel.x, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  currentCmdVel.w =
      constrain(currentCmdVel.w, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  robot.setCmdVel(currentCmdVel);
}