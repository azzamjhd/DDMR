#include "base.h"
#include <PID_v1.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>



// Right motor position Set Point
double posSPR = 500;
// Left motor position Set Point
double posSPL = 500; 
// PID input and output
double posInR, posOutR, posInL, posOutL;
double Kp = 2, Ki = 5, Kd = 1;
// PID objects
PID rightPositionPID(&posInR, &posOutR, &posSPR, Kp, Ki, Kd, DIRECT);
PID leftPositionPID(&posInL, &posOutL, &posSPL, Kp, Ki, Kd, DIRECT);

// Encoder and Motor objects
MotorEncoder rightMotor(PWM_A, A_IN_1, A_IN_2, R_ENC_A, R_ENC_B);
MotorEncoder leftMotor(PWM_B, B_IN_1, B_IN_2, L_ENC_A, L_ENC_B);


void setup() {
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), []() { rightMotor.update(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), []() { leftMotor.update(); }, CHANGE);

    rightPositionPID.SetMode(AUTOMATIC);
    leftPositionPID.SetMode(AUTOMATIC);
}

void loop() {
    Serial.print(rightMotor.getCount()); Serial.print("\t");
    Serial.println(leftMotor.getCount());

    posInR = rightMotor.getCount();
    posInL = leftMotor.getCount();
    rightPositionPID.Compute();
    leftPositionPID.Compute();

    rightMotor.setSpeed(posOutR);
    leftMotor.setSpeed(posOutL);
}