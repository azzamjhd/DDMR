#ifndef DEFINE_BASE_H
#define DEFINE_BASE_H

#include <Arduino.h>
#include <PID_v1.h>
#include <timer_api.hpp>

/************************************
 * MPU6050
 * INT - GND - SDA - SCL - GND - VCC
*************************************/

// Interrupt pin
#define INT 19
// MPU6050 SDA pin
#define SDA 20
// MPU6050 SCL pin
#define SCL 21

/************************************
 * Ultrasonic Sensor HC-SR04
*************************************/
#define ECHO_PIN_1 34
#define TRIG_PIN_1 36
#define ECHO_PIN_2 38
#define TRIG_PIN_2 40
#define ECHO_PIN_3 42
#define TRIG_PIN_3 44

/****************************************************************
 * DC Motor with TB6612FNG Driver
 * inA = attached to Right motor, inB = attached to Left motor
****************************************************************/

// Motor direction
#define _FORWARD 0b01
#define _BACKWARD 0b10
#define _STOP 0b00

// Right Motor Encoder A pin
#define R_ENC_A 3
// Right Motor Encoder B pin
#define R_ENC_B 5
// Left Motor Encoder A pin
#define L_ENC_A 2
// Left Motor Encoder B pin
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


#endif // DEFINE_BASE_H