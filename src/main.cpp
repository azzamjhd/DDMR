#include <Arduino.h>
#include <PID_v1.h>
#include "pins.h"


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
Encoder rightEncoder(R_ENC_A, R_ENC_B);
Encoder leftEncoder(L_ENC_A, L_ENC_B);
Motor rightMotor(PWM_A, A_IN_1, A_IN_2);
Motor leftMotor(PWM_B, B_IN_1, B_IN_2);


void setup() {
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), []() { rightEncoder.update(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), []() { leftEncoder.update(); }, CHANGE);

    rightPositionPID.SetMode(AUTOMATIC);
    leftPositionPID.SetMode(AUTOMATIC);
}

void loop() {
    Serial.print(rightEncoder.getCount()); Serial.print("\t");
    Serial.println(leftEncoder.getCount());

    posInR = rightEncoder.getCount();
    posInL = leftEncoder.getCount();
    rightPositionPID.Compute();
    leftPositionPID.Compute();

    rightMotor.setSpeed(posOutR);
    leftMotor.setSpeed(posOutL);
}