#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <Arduino.h>
#include "motor_driver.hpp"
#include "timer_api.hpp"

typedef struct {
    float x;
    float y;
    float theta;
} Pose;

typedef struct {
    float x;
    float w;
} CmdVel;

class MotorController {
    private:
        MotorDriver *_rightMotor;
        MotorDriver *_leftMotor;
        TimerAPI _motorUpdateTimer;

        Pose _pose;
        CmdVel _cmdVel;
        float _dist_between_wheels;
        float _prev_right_dist;
        float _prev_left_dist;

        int MOTOR_RUN_FREQ = 10; // in Hz

        /// @brief Compute the pose of the robot using odometry data
        void _computePose();
        /// @brief Compute the wheel speeds of the robot using the cmd_vel data for each motor
        void _computeWheelSpeeds();
    public:
        /**
         * @brief Construct a new Motor Controller object
         * 
         * @param rightMotor Right motor driver
         * @param leftMotor Left motor driver
         * @param distBetweenWheels Distance between the wheels in meters
         */
        MotorController(MotorDriver *rightMotor, MotorDriver *leftMotor, float distBetweenWheels);
        /**
         * @brief Get the Pose object
         * @param pose Pose object. pose = {x, y, theta}. x is the x-coordinate in meters, y is the y-coordinate in meters, and theta is the orientation in radians
         */
        void getPose(Pose &pose) const;
        /**
         * @brief print the pose of the robot without the motor running
         */
        void calibrate();
        /**
         * @brief Set the Cmd Vel object
         * @param cmdVel Cmd Vel object = {x, w}. x is the linear velocity in m/s and w is the angular velocity in rad/s
         */
        void setCmdVel(CmdVel cmdVel);
        /// @brief Get current cmd_vel
        /// @param cmdVel 
        void getCmdVel(CmdVel &cmdVel) const;
        /// @brief Reset the motor controller
        void reset();
        /// @brief Run the motor controller
        void run();
        /// @brief Print the pose of the robot
        void printPose(void) const;
        /// @brief Move the robot forward at FORWARD_VELOCITY m/s
        void moveForward();
        /// @brief Move the robot backward at BACKWARD_VELOCITY m/s
        void moveBackward();
        /// @brief Turn the robot left at TURN_VELOCITY rad/s
        void turnLeft();
        /// @brief Turn the robot right at TURN_VELOCITY rad/s
        void turnRight();
        /// @brief Stop the robot
        void stop();
        /**
         * @brief Move the robot using open loop control
         * 
         * @param leftPWM PWM value for the left motor. Range: -255 to 255
         * @param rightPWM PWM value for the right motor. Range: -255 to 255
         */
        void moveOpenLoop(int leftPWM, int rightPWM);
        /**
         * @brief Get the Motor Data object
         * 
         * @param rightMotorData 
         * @param leftMotorData 
         */
        void getMotorData(MotorData &rightMotorData, MotorData &leftMotorData) const;
        /// @brief Reset the pose of the robot
        void resetPose();
        /**
         * @brief Set the PID gains of the motors
         * 
         * @param pidGains PID gains. pidGains = {Kp, Ki, Kd}
         */
        void setPIDGains(PIDGains pidGains);
        /**
         * @brief Get the PID gains of the motors
         * 
         * @return PIDGains = {Kp, Ki, Kd}
         */
        PIDGains getPIDGains(void) const;
        /**
         * @brief Set the Max Velocity object
         * 
         * @param maxVelocity Maximum velocity in m/s
         */
        void setMaxVelocity(float maxVelocity);
        /**
         * @brief Set the Motor Freq object
         * 
         * @param freq Frequency in Hz
         */
        void setMotorFreq(int freq);
};

#endif // MOTOR_CONTROLLER_HPP