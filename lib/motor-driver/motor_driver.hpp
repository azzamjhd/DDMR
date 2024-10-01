/**
 * @file motor_driver.hpp
 * @author azzamjhd (azzamujahid214@gmail.com)
 * @brief Motor driver class for open loop and closed loop control
 * @version 0.1
 * @date 2024-10-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <Arduino.h>
#include <PID_v1.h>
#include "encoder.hpp"

/// @brief Minimum interval for data reading
const unsigned long MIN_INTERVAL = 10;

/// @brief PID gains structure
typedef struct {
    double Kp;
    double Ki;
    double Kd;
} PIDGains;

/// @brief Motor data structure
typedef struct {
    float rpm;
    float angularVelocity;
    float velocity;
    float distance;
    float angle;
} MotorData;

enum class MotorDirection { CW, CCW, STOP };
enum class MotorMode { OPEN_LOOP, CLOSED_LOOP };

/// @brief Motor driver class
class MotorDriver {
   private:
    uint8_t _pinEn;
    uint8_t _pinIn1;
    uint8_t _pinIn2;

    float _wheelRadius;
    int _countPerRev;
    bool _reverse;
    float _MAX_VELOCITY = 1.0; // m/s

    static const int RPM_BUFFER_SIZE = 5;
    float _rpmBuffer[RPM_BUFFER_SIZE];
    int _rpmIndex = 0;
    float _rpmSum = 0;

    MotorData _motorData;
    int _lastEncoderReading;
    unsigned long _lastDataReadingTime;
    float _lastRPM;

    double _setpoint, _input, _output;
    PIDGains _pidGains;
    PID _pid = PID(&_input, &_output, &_setpoint, 0, 0, 0, P_ON_M, DIRECT);

    MotorDirection _motorDir;
    MotorMode _motorMode;
    Encoder *_encoder;
    uint8_t _pwm;
   public:

    /**
     * @brief Consturctor for open loop motor driver
     * 
     * @param pinEn Enable pin
     * @param pinIn1 Direction pin 1
     * @param pinIn2 Direction pin 2
     * @param reverse Reverse direction flah
     */
    MotorDriver(uint8_t pinEn, uint8_t pinIn1, uint8_t pinIn2, bool reverse = false);
    
    /**
     * @brief Constructor for closed loop motor driver
     * 
     * @param pinEn Enable pin
     * @param pinIn1 Direction pin 1
     * @param pinIn2 Direction pin 2
     * @param encoder Encoder object
     * @param wheelRadius Radius of the wheel in meters
     * @param countPerRev Count per revolution of the encoder
     * @param reverse Reverse direction flag
     */
    MotorDriver(uint8_t pinEn, uint8_t pinIn1, uint8_t pinIn2, Encoder *encoder, float wheelRadius, int countPerRev, bool reverse = false);

    /// @brief Run the motor driver
    void run(void);
    /// @brief Print the status of the motor driver
    void printStatus(void);
    /// @brief Reset the motor driver
    void reset(void);

    /// @brief Set the velocity of the motor
    /// @param velocity Velocity in m/s
    void setVelocity(float velocity);
    /// @brief Set the PWM of the motor
    /// @param pwm PWM value
    /// @param mode Motor mode
    void setPWM(int pwm, MotorMode mode = MotorMode::OPEN_LOOP);
    /// @brief Set the mode of the motor
    /// @param mode Motor mode
    void setMode(MotorMode mode);
    /// @brief Set the PID gains of the motor
    /// @param pidGains PID gains
    void setPIDGains(PIDGains &pidGains);
    /// @brief Set the maximum velocity of the motor
    /// @param maxVelocity Maximum velocity in m/s
    void setMaxVelocity(float maxVelocity);

    /// @brief Get the motor data
    /// @param motorData Motor data
    void getMotorData(MotorData &motorData);
    /// @brief Get the wheel radius
    /// @return Wheel radius in meters
    float getWheelRadius(void);
    /// @brief Get the count per revolution
    /// @return Count per revolution
    int getCountPerRev(void);
    /// @brief Get the PID gains
    /// @return PID gains structure
    PIDGains getPIDGains(void);

   private:
    /// @brief Send PWM signal to the motor
    void _sendPWM(void);
    /// @brief Set the direction of the motor
    /// @param dir Motor direction
    void _setDirection(MotorDirection dir);
    /// @brief Initialize the pins
    void _initPins(void);
    /// @brief Compute the RPM of the motor
    /// @return RPM value of the motor
    float _computeRPM(void);
    /// @brief Compute the angular velocity of the motor
    /// @param rpm RPM value of the motor
    /// @return Angular velocity of the motor in rad/s
    float _computeAngularVelocity(float rpm);
    /// @brief Compute the velocity of the motor
    /// @param angularVelocity Angular velocity of the motor
    /// @return Velocity of the motor in m/s
    float _computeVelocity(float angularVelocity);
    /// @brief Compute the distance travelled by the motor
    /// @return Distance travelled by the motor in meters
    float _computeDistance(void);
    /// @brief Compute the angle rotated by the motor
    /// @return Angle rotated by the motor in radians
    float _computeWheelAngle(void);
    /// @brief Compute the motor data
    void _computeMotorData(void);
};

#endif // MOTOR_DRIVER_HPP