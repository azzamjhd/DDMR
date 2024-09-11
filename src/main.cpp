#include <Arduino.h>
#include "pins.h"

int speed = 0;

#define Ultrasonic1 12

void setup() {
    Serial.begin(9600);
    setupMotorPins();
}

void loop() {
    speed++;
    Serial.println(speed);
    setMotorDir('R', 'F');
    setMotorDir('L', 'F');
    analogWrite(PWM_A, speed%255);
    analogWrite(PWM_B, speed%255);
    delay(100);
}