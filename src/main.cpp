#include "base.h"
#include <PID_v1.h>

#define WHELL_DIAMETER 0.065
double Kp = 2, Ki = 1, Kd = 0.1;

// Encoder and Motor objects
MotorEncoder rightMotor(PWM_A, A_IN_1, A_IN_2, R_ENC_A, R_ENC_B, WHELL_DIAMETER, 330);
MotorEncoder leftMotor(PWM_B, B_IN_1, B_IN_2, L_ENC_A, L_ENC_B, WHELL_DIAMETER, 330, true);

DDMRobot robot(&rightMotor, &leftMotor, 0.15);

MotorData rightMotorData, leftMotorData;
Pose pose;

void setup() {
    Serial.begin(115200);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), []() { rightMotor.update(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), []() { leftMotor.update(); }, CHANGE);

    rightMotor.setMaxVelocity(1.0);
    leftMotor.setMaxVelocity(1.0);

    robot.updateMotorGains(Kp, Ki, Kd);
    robot.move(0.01, 0);

    rightMotor.set_velocity(0);
    leftMotor.set_velocity(0.1);
}

void loop() {
    rightMotor.run();
    leftMotor.run();
    rightMotor.getMotorData(rightMotorData);
    leftMotor.getMotorData(leftMotorData);

    // robot.run();
    // robot.getMotorData(rightMotorData, leftMotorData);
    // robot.getPose(pose);

    Serial.print(leftMotorData.velocity*100); Serial.print("\t");
    Serial.print(rightMotorData.velocity*100); Serial.print("\t");
    // Serial.print(pose.x); Serial.print("\t");
    // Serial.print(pose.y); Serial.print("\t");
    // Serial.print(pose.theta); Serial.print("\t");

    Serial.println();
}