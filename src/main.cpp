#include <Arduino.h>
#include "pins.h"

int speed = 0;
bool isForward = true;

// Encoder and Motor objects
Encoder rightEncoder(R_ENC_A, R_ENC_B);
Encoder leftEncoder(L_ENC_A, L_ENC_B);
Motor rightMotor(PWM_A, A_IN_1, A_IN_2);
Motor leftMotor(PWM_B, B_IN_1, B_IN_2);

void setup() {
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), []() { rightEncoder.update(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), []() { leftEncoder.update(); }, CHANGE);
}

void loop() {
    if (speed < 256 && isForward) {
        speed++;
    } else if ( speed < 256 && !isForward) {
        speed--;
    }

    if (speed == 255) {
        isForward = false;
        rightMotor.setDir(STOP);
    } else if (speed == 0) {
        isForward = true;
    }

    Serial.print("Speed: "); Serial.println(speed);
    Serial.print("Right Encoder: "); Serial.println(rightEncoder.getCount());
    Serial.print("Left Encoder: "); Serial.println(leftEncoder.getCount());

    rightMotor.setSpeed(speed);
    leftMotor.setSpeed(speed);
}