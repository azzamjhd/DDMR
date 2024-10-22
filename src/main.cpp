#include <config.h>
#include <motor_driver.hpp>
#include "serial_command.hpp"

#include <PID_v1.h>
#include <Ultrasonic.h>

// #define TEST_DRIVER
// #define TEST_CONTROLLER
#define TEST_CARTESIAN_CONTROL

#define WHELL_DIAMETER 0.065
#define COUNT_PER_REV 320
double Kp = 20, Ki = 400, Kd = 0;
PIDGains pidGains = {Kp, Ki, Kd};
MotorData rightMotorData, leftMotorData;

Ultrasonic ultrasonic1(TRIG_PIN_1, ECHO_PIN_1);
Ultrasonic ultrasonic2(TRIG_PIN_2, ECHO_PIN_2);
Ultrasonic ultrasonic3(TRIG_PIN_3, ECHO_PIN_3);

Encoder rightEncoder(R_ENC_A, R_ENC_B);
Encoder leftEncoder(L_ENC_A, L_ENC_B,true);

MotorDriver rightMotor(PWM_A, A_IN_1, A_IN_2, &rightEncoder, WHELL_DIAMETER, COUNT_PER_REV);
MotorDriver leftMotor(PWM_B, B_IN_1, B_IN_2, &leftEncoder, WHELL_DIAMETER, COUNT_PER_REV);

#include "motor_controller.hpp"
#define DIST_BETWEEN_WHEELS 0.2
MotorController robot(&rightMotor, &leftMotor, DIST_BETWEEN_WHEELS);
SerialCommand serialCommand(&robot);
CmdVel cmdVel;

// Odometry controls
double distanceError = 0;
double velOutput = 0;
double Kp_pos = 1, Ki_pos = 0, Kd_pos = 0;
PID CartesianControlPID(&distanceError, &velOutput, 0, Kp, Ki, Kd, DIRECT);
double angleError = 0; 
double angleCmdOutput = 0;
double Kp_angle = 1, Ki_angle = 0, Kd_angle = 0;
PID AngleControlPID(&angleError, &angleCmdOutput, 0, Kp, Ki, Kd, DIRECT);

void rightEncoderISR() { rightEncoder.count_isr(); }
void leftEncoderISR() { leftEncoder.count_isr(); }

void setupInterrupts(void) {
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, RISING);
}

const int numReadings = 5;
static int readings1[numReadings];      // the readings from the ultrasonic sensor 1
static int readings2[numReadings];      // the readings from the ultrasonic sensor 2
static int readings3[numReadings];      // the readings from the ultrasonic sensor 3
static int readIndex = 0;               // the index of the current reading
static int total1 = 0;                  // the running total of sensor 1
static int total2 = 0;                  // the running total of sensor 2
static int total3 = 0;                  // the running total of sensor 3
static int average1 = 0;                // the average of sensor 1
static int average2 = 0;                // the average of sensor 2
static int average3 = 0;                // the average of sensor 3

void ultrasonicRead() {

    // subtract the last reading:
    total1 = total1 - readings1[readIndex];
    total2 = total2 - readings2[readIndex];
    total3 = total3 - readings3[readIndex];

    // read from the sensor:
    readings1[readIndex] = ultrasonic1.read();
    readings2[readIndex] = ultrasonic2.read();
    readings3[readIndex] = ultrasonic3.read();

    // add the reading to the total:
    total1 = total1 + readings1[readIndex];
    total2 = total2 + readings2[readIndex];
    total3 = total3 + readings3[readIndex];

    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
    }

    // calculate the average:
    // average1 = total1 / numReadings;
    // average2 = total2 / numReadings;
    // average3 = total3 / numReadings;

    average1 = ultrasonic1.read();
    average2 = ultrasonic2.read();
    average3 = ultrasonic3.read();


    // Serial.print(">");
    // Serial.print("S1:"); Serial.print(average1); Serial.print(",");
    // Serial.print("S2:"); Serial.print(average2); Serial.print(",");
    // Serial.print("S3:"); Serial.print(average3); Serial.print(",");
    // Serial.println();
}

void autoRun() {
    ultrasonicRead();

    const int safeDistance = 20; // Safe distance in cm
    const int followDistance = 15; // Distance to maintain from the wall in cm

    if (average2 < safeDistance && average1 < safeDistance && average3 < safeDistance) {
        // All sensors blocked, move backward and turn in place
        robot.setCmdVel({-0.5, 5});
    } else if (average2 < safeDistance) {
        // Front sensor blocked, turn in place
        robot.setCmdVel({0, 3});
    } else if (average1 < followDistance) {
        // Right side too close to the wall, turn left slightly
        robot.setCmdVel({0.3, 3});
    } else if (average3 < followDistance) {
        // Left side too close to the wall, turn right slightly
        robot.setCmdVel({0.3, -3});
    } else {
        // Move forward
        robot.setCmdVel({0.5, 0});
    }
}

void setup() {
    Serial.begin(115200);
    setupInterrupts();
    robot.setPIDGains(pidGains);

    CartesianControlPID.SetMode(AUTOMATIC);
    CartesianControlPID.SetOutputLimits(-0.5, 0.5);
    AngleControlPID.SetMode(AUTOMATIC);
    AngleControlPID.SetOutputLimits(-2, 2);

#ifdef TEST_DRIVER

    rightMotor.setPIDGains(pidGains);
    leftMotor.setPIDGains(pidGains);

    rightMotor.setVelocity(1);
    leftMotor.setVelocity(1);

#endif
}

bool print = true;
Pose currentPose;

void moveTo(float x_target, float y_target, Pose currentPose) {
    distanceError = sqrt(pow(x_target - currentPose.x, 2) + pow(y_target - currentPose.y, 2));
    angleError = atan2(y_target - currentPose.y, x_target - currentPose.x) - currentPose.theta;

    while (angleError > PI) angleError -= 2*PI;
    while (angleError < -PI) angleError += 2*PI;

    CartesianControlPID.Compute();
    AngleControlPID.Compute();

    robot.setCmdVel({float(velOutput), float(angleCmdOutput)});
}

// ====================== MAIN LOOP ======================

void loop() {
#ifdef TEST_DRIVER
    // rightMotor.printStatus(); Serial.print("\t");
    // leftMotor.printStatus(); Serial.print("\t");
    // rightEncoder.printCount(); Serial.print("\t");
    // leftEncoder.printCount(); Serial.println();
    rightMotor.getMotorData(rightMotorData);
    leftMotor.getMotorData(leftMotorData);

    Serial.print(">");
    Serial.print("R:"); Serial.print(rightMotorData.velocity); Serial.print(",");
    Serial.print("L:"); Serial.print(leftMotorData.velocity); Serial.print(",");
    Serial.println();

    // rightMotor.setPWM(255);
    // leftMotor.setPWM(255);

    // rightMotor.setVelocity(1);
    // leftMotor.setVelocity(1);

    rightMotor.run();
    leftMotor.run();

#elif defined(TEST_CONTROLLER)
    // autoRun();

    robot.run();
    // serialCommand.readSerial();
    robot.printPose();
    rightMotor.getMotorData(rightMotorData);
    leftMotor.getMotorData(leftMotorData);

    if (Serial.available()) {
        String input = Serial.readStringUntil('\n'); // Read the input until newline character
        Serial.print("Input: "); Serial.println(input); // Print the inputted data back

        if (input.length() > 0) {

            int x, w;
            if (sscanf(input.c_str(), "c %d %d", &x, &w) == 2) {
                cmdVel.x = float(x)/1000.0;
                cmdVel.w = float(w)/1000.0;
                robot.setCmdVel(cmdVel);
                // Serial.print("Set cmdVel to x: "); Serial.print(x); Serial.print(", w: "); Serial.println(w);
            } else {
                Serial.println("Invalid input format. Please enter two integers separated by a space.");
            }
        }
    }

    // Serial.print(">");
    // Serial.print("R:"); Serial.print(rightMotorData.velocity); Serial.print(",");
    // Serial.print("L:"); Serial.print(leftMotorData.velocity); Serial.print(",");
    // Serial.println();

#elif defined(TEST_CARTESIAN_CONTROL)
    robot.run();
    robot.getPose(currentPose);
    moveTo(1, 1, currentPose);

    robot.printPose();
#endif
}