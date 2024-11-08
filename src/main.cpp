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
#define DIST_BETWEEN_WHEELS 0.18

double Kp = 20, Ki = 400, Kd = 0;
PIDGains pidGains = {Kp, Ki, Kd};
MotorData rightMotorData, leftMotorData;

Ultrasonic ultrasonic1(TRIG_PIN_1, ECHO_PIN_1);
Ultrasonic ultrasonic2(TRIG_PIN_2, ECHO_PIN_2);
Ultrasonic ultrasonic3(TRIG_PIN_3, ECHO_PIN_3);

Encoder rightEncoder(R_ENC_A, R_ENC_B);
Encoder leftEncoder(L_ENC_A, L_ENC_B,true);

MotorDriver rightMotor(PWM_A, A_IN_1, A_IN_2, &rightEncoder, WHELL_DIAMETER/2, COUNT_PER_REV);
MotorDriver leftMotor(PWM_B, B_IN_1, B_IN_2, &leftEncoder, WHELL_DIAMETER/2, COUNT_PER_REV);

#include "motor_controller.hpp"
MotorController robot(&rightMotor, &leftMotor, DIST_BETWEEN_WHEELS);
SerialCommand serialCommand(&robot);
CmdVel cmdVel;

// Odometry controls
double distanceError = 0;
double velOutput = 0;
double Kp_pos = 1.5, Ki_pos = 0.01, Kd_pos = 0.5;
PID CartesianControlPID(&distanceError, &velOutput, 0, Kp_pos, Ki_pos, Kd_pos, DIRECT);
double angleInput = 0; 
double angleCmdOutput = 0;
double angleSetpoint = 0;
double Kp_angle = 2, Ki_angle = 0, Kd_angle = 0.5;
PID AngleControlPID(&angleInput, &angleCmdOutput, &angleSetpoint, Kp_angle, Ki_angle, Kd_angle, DIRECT);

void rightEncoderISR() { rightEncoder.count_isr(); }
void leftEncoderISR() { leftEncoder.count_isr(); }

void setupInterrupts(void) {
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, RISING);
}

void setup() {
    Serial.begin(115200);
    setupInterrupts();
    robot.setPIDGains(pidGains);

    CartesianControlPID.SetMode(AUTOMATIC);
    CartesianControlPID.SetOutputLimits(-0.3, 0.3);
    AngleControlPID.SetMode(AUTOMATIC);
    AngleControlPID.SetOutputLimits(-3, 3);
}

bool print = true;
Pose currentPose;

void moveTo(float x_target, float y_target, Pose currentPose) {
    CmdVel currentCmdVel;
    distanceError = sqrt(pow(x_target - currentPose.x, 2) + pow(y_target - currentPose.y, 2));
    angleSetpoint = atan2(y_target - currentPose.y, x_target - currentPose.x);
    while (angleSetpoint > PI) angleSetpoint -= 2*PI;
    while (angleSetpoint < -PI) angleSetpoint += 2*PI;
    angleSetpoint = angleSetpoint * 180.0 / PI;

    angleInput = currentPose.theta * 180.0 / PI;

    if (fabs(angleSetpoint-angleInput) > 5) {
        AngleControlPID.Compute();

        currentCmdVel.w = angleCmdOutput;
        currentCmdVel.x = 0;
        robot.setCmdVel(currentCmdVel);
    } else {
        CartesianControlPID.Compute();
        currentCmdVel.x = -velOutput;
        currentCmdVel.w = 0;
        // v = velOutput;
        robot.setCmdVel(currentCmdVel);
    }
    // robot.setCmdVel(currentCmdVel);

    Serial.print("ERR: "); 
    Serial.print(distanceError); Serial.print("\t");
    Serial.print(angleSetpoint); Serial.print("\t");
    Serial.print("CMD: ");
    Serial.print(currentCmdVel.x); Serial.print("\t");
    Serial.print(currentCmdVel.w); Serial.print("\t");

}

float MAX_LINEAR_VELOCITY = 0.5;
float MAX_ANGULAR_VELOCITY = 3.0;

float usePID(float setpoint) {
    angleSetpoint = setpoint;
    angleInput = currentPose.theta;
    AngleControlPID.Compute();
    return angleCmdOutput;
}

void goToGoal(Pose currentPose, float x_target, float y_target, float theta_target = NAN) {
    CmdVel currentCmdVel;

    bool use_beta = false;
    float k_rho = 0.5;
    float k_alpha = 2.0;
    float k_beta = 1.0;
    // double k_p(2.0), k_i(0.01), k_d(0.1);
    float goal_tolerance = 0.1;

    float alpha, beta;

    float dx = x_target - currentPose.x;
    float dy = y_target - currentPose.y;
    float rho = sqrt(dx*dx + dy*dy);
    alpha = atan2(dy, dx) - currentPose.theta;
    alpha = atan2(sin(alpha), cos(alpha));

    Serial.print(alpha); Serial.print("\t");

    if (theta_target != NAN) {
        beta = theta_target - currentPose.theta;
        beta = atan2(sin(beta), cos(beta));
        use_beta = true;
    } else {
        beta = 0;
        use_beta = false;
    }

    enum State { MOVE_TO_GOAL, ADJUST_ANGLE };
    static State state = MOVE_TO_GOAL;

    if (rho <= goal_tolerance) {
        state = ADJUST_ANGLE;
    }

    switch (state) {
        case MOVE_TO_GOAL:
            currentCmdVel.x = k_rho * rho;
            currentCmdVel.w = k_alpha * alpha;
            break;
        case ADJUST_ANGLE:
            if (use_beta) {
                // currentCmdVel.w = usePID(theta_target);
                currentCmdVel.x = 0;
                currentCmdVel.w = k_beta * beta;
            } else {
                currentCmdVel.x = 0;
                currentCmdVel.w = 0;
            }
            break;
    }

    // AngleControlPID.Compute();
    currentCmdVel.x = constrain(currentCmdVel.x, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    // currentCmdVel.w = angleCmdOutput;
    currentCmdVel.w = constrain(currentCmdVel.w, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    robot.setCmdVel(currentCmdVel);
}

// ====================== MAIN LOOP ======================

void loop() {
#ifdef TEST_DRIVER
    robot.calibrate();

    rightMotor.getMotorData(rightMotorData);
    leftMotor.getMotorData(leftMotorData);

    // Calibrate how many counts per meter

    Serial.print("Enc: ");
    rightEncoder.printCount(); Serial.print(", ");
    leftEncoder.printCount(); Serial.print("\t");

    Serial.print("Dist: ");
    Serial.print(rightMotorData.distance); Serial.print(", ");
    Serial.print(leftMotorData.distance); Serial.print("\t");

    robot.printPose();

    // Serial.print(">");
    // Serial.print("R:"); Serial.print(rightMotorData.velocity); Serial.print(",");
    // Serial.print("L:"); Serial.print(leftMotorData.velocity); Serial.print(",");
    // Serial.println();

    // rightMotor.setPWM(255);
    // leftMotor.setPWM(255);

    // rightMotor.setVelocity(1);
    // leftMotor.setVelocity(1);

    // rightMotor.run();
    // leftMotor.run();

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
    robot.getPose(currentPose);
    // moveTo(1, 1, currentPose);
    goToGoal(currentPose, 1, 1);

    robot.run();
    robot.printPose();
#endif
}