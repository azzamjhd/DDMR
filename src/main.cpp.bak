#include "base.h"
#include <PID_v1.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// Right motor position Set Point
double posSPR = 2000;
// Left motor position Set Point
double posSPL = -2000; 
// PID input and output
double posInR, posOutR, posInL, posOutL;
double Kp = 1, Ki = 1, Kd = 0.1;
// PID objects
PID rightPositionPID(&posInR, &posOutR, &posSPR, Kp, Ki, Kd, P_ON_M, DIRECT);
PID leftPositionPID(&posInL, &posOutL, &posSPL, Kp, Ki, Kd, P_ON_M, DIRECT);

// Encoder and Motor objects
MotorEncoder rightMotor(PWM_A, A_IN_1, A_IN_2, R_ENC_A, R_ENC_B);
MotorEncoder leftMotor(PWM_B, B_IN_1, B_IN_2, L_ENC_A, L_ENC_B);

Ultrasonic ultrasonic1(TRIG_PIN_1, ECHO_PIN_1);
Ultrasonic ultrasonic2(TRIG_PIN_2, ECHO_PIN_2);
Ultrasonic ultrasonic3(TRIG_PIN_3, ECHO_PIN_3);

int PWMOffset(int offset, int value) {
    //
    int result;
    if (value < 0 ) {
        result = (value + 255) / 255 * (255 - offset) - offset;
    } else if (value > 0) {
        result = value / 255 * (255 - offset) + offset;
    } else {
        result = 0;
    }
    return result;
}

void positionControl() {
    posInR = rightMotor.getCount();
    posInL = -leftMotor.getCount();
    rightPositionPID.Compute();
    leftPositionPID.Compute();
    // rightMotor.setSpeed(PWMOffset(10, posOutR));
    // leftMotor.setSpeed(PWMOffset(10, posOutL));
    rightMotor.setSpeed(posOutR);
    leftMotor.setSpeed(posOutL);

    Serial.print(posOutR); Serial.print("\t");
    Serial.print(posOutL); Serial.println("\t");
}

void setup() {
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), []() { rightMotor.update(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), []() { leftMotor.update(); }, CHANGE);

    rightPositionPID.SetMode(AUTOMATIC);
    rightPositionPID.SetOutputLimits(-255, 255);
    leftPositionPID.SetMode(AUTOMATIC);
    leftPositionPID.SetOutputLimits(-255, 255);

    rightMotor.setCountPerRev(330);
    leftMotor.setCountPerRev(330);
}

int currentCount, lastCount;
unsigned long currentTime, lastTime;

void loop() {
    positionControl();
    Serial.print(rightMotor.getCount()); Serial.print("\t");
    Serial.print(leftMotor.getCount()); Serial.print("\t");
    // Serial.print(rightMotor.getSpeed()); Serial.print("\t");
    // Serial.print(-leftMotor.getSpeed()); Serial.println("\t");

    // rightMotor.setSpeed(100);
    // leftMotor.setSpeed(100);
    
    // Serial.print("\t1:" + String(ultrasonic1.getDistance()) + "\t");
    // Serial.print("\t2:" + String(ultrasonic2.getDistance()) + "\t");
    // Serial.print("\t3:" + String(ultrasonic3.getDistance()) + "\t");

    // print speed
    // currentCount = rightMotor.getCount();
    // currentTime = millis();

    // if (currentTime - lastTime > 1000) {
    //     float speed = (currentCount - lastCount) / 330.0 * 60;
    //     Serial.println(speed);
    //     lastCount = currentCount;
    //     lastTime = currentTime;
    // }

}