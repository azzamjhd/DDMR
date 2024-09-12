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

// ==================== DC Motor with TB6612FNG Driver ====================
// inA = attached to Right motor, inB = attached to Left motor

// Rotary Encode Motor
#define FORWARD 0b01
#define BACKWARD 0b10
#define STOP 0b00

// Right Motor Encoder A pin
#define R_ENC_A 3
#define R_ENC_B 5
// Left Motor Encoder A pin
#define L_ENC_A 2
#define L_ENC_B 4


// Right motor PWM pin
#define PWM_A 11
// Right motor IN1
#define A_IN_2 10
// Right motor IN2
#define A_IN_1 9
// Left motor IN1
#define B_IN_1 8
// Left motor IN2
#define B_IN_2 7
// Left motor PWM pin
#define PWM_B 6

// MPU6050
// INT - GND - SDA - SCL - GND - VCC

// Interrupt pin
#define INT 19
// MPU6050 SDA pin
#define SDA 20
// MPU6050 SCL pin
#define SCL 21


class Encoder {
private:
    int pinA;
    int pinB;
    int count;
    uint8_t curretState, lastState;

public:
    Encoder(int pinA, int pinB) {
        this->pinA = pinA;
        this->pinB = pinB;
        this->count = 0;
        this->curretState = 0;
        this->lastState = 0;
        pinMode(pinA, INPUT);
        pinMode(pinB, INPUT);
    }

    void update() {
        this->curretState = digitalRead(this->pinA) | digitalRead(this->pinB) << 1;
        if (this->curretState == 0b10 && this->lastState == 0b01) {
            this->count++;
        } else if (this->curretState == 0b11 && this->lastState == 0b00) {
            this->count--;
        }
    }

    int getCount() {
        return this->count;
    }
};

class Motor {
private:
    int pwm;
    int in1;
    int in2;
public:
    Motor(int pwm, int in1, int in2) {
        this->pwm = pwm;
        this->in1 = in1;
        this->in2 = in2;
        pinMode(pwm, OUTPUT);
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
    }

    void setSpeed(int speed) {
        analogWrite(this->pwm, speed);
    }

    void setDir(uint8_t dir) {
        switch (dir) {
            case FORWARD:
                digitalWrite(this->in1, LOW);
                digitalWrite(this->in2, HIGH);
                break;
            case BACKWARD:
                digitalWrite(this->in1, HIGH);
                digitalWrite(this->in2, LOW);
                break;
            case STOP:
                digitalWrite(this->in1, LOW);
                digitalWrite(this->in2, LOW);
                break;
        }
    }
};

#endif // PINS_H_