#ifndef PINS_H_
#define PINS_H_

#include <Arduino.h>

// Ultrasonic Sensor
#define ECHO_PIN_1 34
#define TRIG_PIN_1 36
#define ECHO_PIN_2 38
#define TRIG_PIN_2 40
#define ECHO_PIN_3 42
#define TRIG_PIN_3 44

// Rotary Encode Motor
#define R_ENC_A 3
#define R_ENC_B 5
#define L_ENC_A 2
#define L_ENC_B 4

// Driver Motor TB6612FNG
// A = Right, B = Left
#define PWM_A 11
#define A_IN_2 10
#define A_IN_1 9
#define B_IN_1 8
#define B_IN_2 7
#define PWM_B 6

// MPU6050
// INT - GND - SDA - SCL - GND - VCC
#define INT 19
#define SDA 20
#define SCL 21

void setupMotorPins() {
  pinMode(PWM_A, OUTPUT);
  pinMode(A_IN_1, OUTPUT);
  pinMode(A_IN_2, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(B_IN_1, OUTPUT);
  pinMode(B_IN_2, OUTPUT);
  Serial.println("Motor pins setup");
}

void setMotorDir(char motor, char dir) {
    switch(motor) {
        case 'R':
            if (dir == 'F') {
                digitalWrite(A_IN_1, HIGH);
                digitalWrite(A_IN_2, LOW);
                Serial.println("Right motor moving forward");
            } else if (dir == 'B') {
                digitalWrite(A_IN_1, LOW);
                digitalWrite(A_IN_2, HIGH);
                Serial.println("Right motor moving backward");
            } else {
                digitalWrite(A_IN_1, LOW);
                digitalWrite(A_IN_2, LOW);
                Serial.println("Right motor stopped");
            }
            break;
        case 'L':
            if (dir == 'F') {
                digitalWrite(B_IN_1, HIGH);
                digitalWrite(B_IN_2, LOW);
                Serial.println("Left motor moving forward");
            } else if (dir == 'B') {
                digitalWrite(B_IN_1, LOW);
                digitalWrite(B_IN_2, HIGH);
                Serial.println("Left motor moving backward");
            } else {
                digitalWrite(B_IN_1, LOW);
                digitalWrite(B_IN_2, LOW);
                Serial.println("Left motor stopped");
            }
            break;
    }
}

#endif // PINS_H_