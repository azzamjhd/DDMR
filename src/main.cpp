#include "base.h"
#include "motor_driver.hpp"
#include <PID_v1.h>

#define WHELL_DIAMETER 0.065
#define COUNT_PER_REV 12
double Kp = 2, Ki = 1, Kd = 0.1;
PIDGains pidGains = {Kp, Ki, Kd};
MotorData motorData;

Encoder rightEncoder(R_ENC_A, R_ENC_B);
Encoder leftEncoder(L_ENC_A, L_ENC_B, true);

MotorDriver rightMotor(PWM_A, A_IN_1, A_IN_2, &rightEncoder, WHELL_DIAMETER, COUNT_PER_REV);
MotorDriver leftMotor(PWM_B, B_IN_1, B_IN_2, &leftEncoder, WHELL_DIAMETER, COUNT_PER_REV, true);

void rightEncoderISR() { rightEncoder.count_isr(); }
void leftEncoderISR() { leftEncoder.count_isr(); }

void setupInterrupts(void) {
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, RISING);
}

void setup() {
    Serial.begin(9600);
    setupInterrupts();

    rightMotor.setPIDGains(pidGains);
    leftMotor.setPIDGains(pidGains);

    rightMotor.setVelocity(0.2);
    leftMotor.setVelocity(0.1);
}

void loop() {
    rightMotor.printStatus();
    leftMotor.printStatus();

    rightMotor.run();
    leftMotor.run();
}