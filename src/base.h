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

typedef struct {
    float rpm;               //< Revolutions per minute (RPM) of the motor.
    float angular_velocity;  //< Angular velocity of the motor (in rad/s).
    float velocity;          //< Linear velocity of the motor (in m/s).
    float distance;          //< Total Traveled distance (in m).
    float angle;             //< Angular position of the motor (in radians).
} MotorData;

enum class MotorMode { OPEN_LOOP, CLOSED_LOOP };
enum class MotorDirection { FORWARD, BACKWARD, STOP };


/** Motor class */
class MotorEncoder {
private:
    bool _reverse;
    int _pwm;
    int _in1, _in2;
    int _pinA, _pinB;
    volatile uint64_t _count;
    uint8_t _curretState, _lastState;
    float _wheelDiameter = 0; // wheel diameter in mm
    int _countPerRev = 0;
    float _MAX_VELOCITY = 1.0; // m/s

    MotorData _motor_data;
    uint64_t _lastCount = 0;
    unsigned long _lastTime = 0;
    

    double _setpoint = 0, _input = 0, _output = 0;
    double _Kp = 0, _Ki = 0, _Kd = 0;
    PID _pid = PID(&_input, &_output, &_setpoint, _Kp, _Ki, _Kd, DIRECT);

    void setDir(MotorDirection);
    float _computeRPM(void);
    float _computeAngularVelocity(float rpm);
    float _computeVelocity(float angular_velocity);
    float _computeDistance(void);
    float _computeAngle(void);
public:
    /**
     * @brief Construct a new Motor Encoder object
     * @param wheelDiameter Wheel diameter in meters
     * @param countPerRev Encoder Count per revolution
     */
    MotorEncoder(int pwm, int in1, int in2, int pinA, int pinB, float wheelDiameter, int countPerRev, bool reverse = false);

    void run(void);
    void reset(void);
    void set_velocity(float);
    void setSpeed(int);
    void setMaxVelocity(float);
    void updateGains(double, double, double);
    void getGains(double&, double&, double&);
    float getWheelDiameter();
    int getCountPerRev();

    // Encoder functions
    void update(void);
    void computeMotorData(void);
    int getCount(void);
    void getMotorData(MotorData&);
};



/** Ultrasonic class */
class Ultrasonic {
private:
    int _echoPin;
    int _trigPin;
    unsigned long _lastTriggerTime;
    unsigned long _echoStartTime;
    unsigned long _timeout;
    bool _waitingForEcho;
public:
    Ultrasonic(int echoPin, int trigPin, unsigned long timeout = 20000);
    float getDistance();
    void trigger();
};



typedef struct {
    float x;
    float y;
    float theta;
} Pose;

typedef struct {
    float x;
    float w;
} CmdVel;

/** DDMRobot class */
class DDMRobot {
private:
    MotorEncoder *_rightMotor;
    MotorEncoder *_leftMotor;
    TimerAPI _motorUpdateTimer = TimerAPI(0);

    Pose _pose;
    CmdVel _cmdVel;
    float _wheelBaseDistance;
    float _prevRightDistance;
    float _prevLeftDistance;
    int _MOTOR_RUN_FREQ = 20;

    void _computePose(void);
    void _computeWheelSpeeds();
public:
    DDMRobot(MotorEncoder*, MotorEncoder*, float);
    void run(void);
    void reset(void);
    void resetPose(void);
    void updateMotorGains(double, double, double);

    void move(float, float);
    void moveOpenLoop(int speedR, int speedL);
    void stop(void);
    void moveForward(void);
    void moveBackward(void);
    void turnLeft(void);
    void turnRight(void);

    void getMotorData(MotorData&, MotorData&);
    void getMotorGains(double&, double&, double&);
    void getPose(Pose&);
};

#endif // DEFINE_BASE_H