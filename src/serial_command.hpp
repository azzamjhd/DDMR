#ifndef SER_CMD_HPP
#define SER_CMD_HPP

#include <Arduino.h>
#include "motor_controller.hpp"

typedef enum {
    FLAG_CLOSE = 'c',        /**< Flag for closed-loop mode */
    FLAG_OPEN = 'o',         /**< Flag for open-loop mode */
    FLAG_POSE = 'q',         /**< Flag to request robot's pose (odometry) */
    FLAG_MOTOR_STATUS = 'm', /**< Flag to request motor status */
    FLAG_RESET = 'r',        /**< Flag to reset the robot's pose*/
    FLAG_PID_GAINS = 'p',    /**< Flag to update PID gains */
    FLAG_PID_GET = 'g'       /**< Flag to request PID gains */
} Flags;

class SerialCommand {
    private:
        MotorController *_robot;

        int parseCommand(const String &cmd) {
            char flag = cmd.charAt(0);

            switch (flag) {
                case FLAG_CLOSE:
                    int x, w;
                    if (sscanf(cmd.c_str(), "c %d %d", &x, &w) == 2) {
                        CmdVel cmdVel;
                        cmdVel.x = x/1000.0;
                        cmdVel.w = w/1000.0;
                        _robot->setCmdVel(cmdVel);
                        return 0;
                    }
                    break;
                
                case FLAG_OPEN:
                    unsigned int pwmR, pwmL;
                    if (sscanf(cmd.c_str(), "o %u %u", &pwmR, &pwmL) == 3) {
                        if (pwmR <= 255 && pwmL <= 255) {
                            _robot->moveOpenLoop(pwmL, pwmR);
                            return 0;
                        } else {
                            return -2;
                        }
                    }
                    break;
                
                case FLAG_POSE:
                    Pose pose;
                    _robot->getPose(pose);
                    Serial.println(String(pose.x) + "," + String(pose.y) + "," + String(pose.theta));
                    return 1;
                    break;

                case FLAG_MOTOR_STATUS:
                    MotorData rightMotorData, leftMotorData;
                    _robot->getMotorData(rightMotorData, leftMotorData);
                    Serial.print(leftMotorData.rpm);                Serial.print(",");
                    Serial.print(leftMotorData.velocity);           Serial.print(",");
                    Serial.print(leftMotorData.angularVelocity);    Serial.print(",");
                    Serial.print(leftMotorData.distance);           Serial.print(",");
                    Serial.print(leftMotorData.angle);              Serial.print(",");
                    Serial.print(rightMotorData.rpm);               Serial.print(",");
                    Serial.print(rightMotorData.velocity);          Serial.print(",");
                    Serial.print(rightMotorData.angularVelocity);   Serial.print(",");
                    Serial.print(rightMotorData.distance);          Serial.print(",");
                    Serial.print(rightMotorData.angle);             Serial.println();

                    return 1;
                    break;

                case FLAG_RESET:
                    _robot->resetPose();
                    return 0;
                    break;

                case FLAG_PID_GAINS:
                    float kp, ki, kd;
                    if (sscanf(cmd.c_str(), "p %f %f %f", &kp, &ki, &kd) == 3) {
                        PIDGains pidGains = {kp, ki, kd};
                        _robot->setPIDGains(pidGains);
                        return 0;
                    } else {
                        return -1;
                    }
                    break;

                case FLAG_PID_GET:
                    PIDGains pidGains = _robot->getPIDGains();
                    Serial.print(pidGains.Kp); Serial.print(",");
                    Serial.print(pidGains.Ki); Serial.print(",");
                    Serial.print(pidGains.Kd); Serial.println();
                    return 1;
                    break;

                default:
                    return -1;
                    break;
            }
            return -1;
        }

        void sendAck(int code) {
            switch (code) {
                case 0:
                    Serial.println("OK");
                    break;
                case 1:
                    break;
                case -1:
                    Serial.println("ERR: Invalid command");
                    break;
                case -2:
                    Serial.println("ERR: PWM values out of range");
                    break;
                default:
                    Serial.println("ERR: Unknown error");
                    break;
            }
        }

    public:
        SerialCommand(MotorController *robot) {_robot = robot;}

        void readSerial() {
            while (Serial.available()) {
            char c = Serial.read();
            static String inputBuffer = "";

            if (c == '\n') {
                int ret = parseCommand(inputBuffer);
                sendAck(ret);
                inputBuffer = "";
            } else {
                inputBuffer += c;
            }
        }
        }
};

#endif