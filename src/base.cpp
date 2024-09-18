#include "base.h"

// ===============================================
// =========== DDMR Class ========================
// ===============================================
DDMRobot::DDMRobot(MotorEncoder * rightMotor, MotorEncoder * leftMotor) {
    this->rightMotor = rightMotor;
    this->leftMotor = leftMotor;
}

/** Set wheel distance between two wheels
 * @param wheelDistance: distance between two wheels in mm
 */
void DDMRobot::setWheelDistance(float wheelDistance) {
    this->wheelDistance = wheelDistance;
}

float DDMRobot::getHeading() {
    /** Heading
     *      Sr - Sl
     * θ = ---------
     *         D
     */
    return (rightMotor->getDistance() - leftMotor->getDistance()) / wheelDistance;
}
float DDMRobot::getAngularSpeed() {
    /** Angular Velocity
     *      Vr - Vl
     * ω = ---------
     *         D
     */
    return (rightMotor->getSpeed() - leftMotor->getSpeed()) / wheelDistance;
}
float DDMRobot::getR_ICR() {
    float rightSpeed = rightMotor->getSpeed();
    float leftSpeed = leftMotor->getSpeed();
    /** Distance from Instantaneous Center of Rotation (ICR) to Center point between wheels
     *      Vr + Vl    D
     * R = -------- * ---
     *      Vr - Vl    2
     */
    float ICR_Radius = (rightSpeed + leftSpeed) / (rightSpeed - leftSpeed) * wheelDistance / 2;
    return ICR_Radius;
}


// ===============================================
// =========== Motor Encoder Class ===============
// ===============================================

/** Constructor for Motor class
 * @param pwm: PWM pin of motor
 * @param in1: IN1 pin of motor
 * @param in2: IN2 pin of motor
 * @param pinA: Encoder A pin
 * @param pinB: Encoder B pin
*/
MotorEncoder::MotorEncoder(int pwm, int in1, int in2, int pinA, int pinB) {
    this->pwm = pwm;
    this->in1 = in1;
    this->in2 = in2;
    this->pinA = pinA;
    this->pinB = pinB;
    this->count = 0;
    this->curretState = 0;
    this->lastState = 0;
    pinMode(pwm, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
}

/** Set direction of motor
 * @param dir: direction of motor _FORWARD, _BACKWARD, _STOP
 */
void MotorEncoder::setDir(uint8_t dir) {
    switch (dir) {
        case _FORWARD:
            digitalWrite(this->in1, LOW);
            digitalWrite(this->in2, HIGH);
            break;
        case _BACKWARD:
            digitalWrite(this->in1, HIGH);
            digitalWrite(this->in2, LOW);
            break;
        case _STOP:
            digitalWrite(this->in1, LOW);
            digitalWrite(this->in2, LOW);
            break;
    }
}

/** Set speed of motor
 * @param speed: speed of motor -255 to 255
 */
void MotorEncoder::setSpeed(int speed) {
    // Serial.print("Speed: " + String(speed) + "\t");
    if (speed > 0) {
        setDir(_FORWARD);
        analogWrite(this->pwm, speed);
    } else if (speed < 0) {
        setDir(_BACKWARD);
        analogWrite(this->pwm, -speed);
    } else {
        setDir(_STOP);
        analogWrite(this->pwm, 0);
        // Serial.print("\tStop\t");
    }
}

/** Set count per revolution of wheel
 * @param countPerRev: count per revolution of wheel
 */
void MotorEncoder::setCountPerRev(float countPerRev) {
    this->countPerRev = countPerRev;
}

/** Set wheel diameter of motor
 * @param wheelDiameter: diameter of wheel in mm
 */
void MotorEncoder::setWheelDiameter(float wheelDiameter) {
    this->wheelDiameter = wheelDiameter;
}

/** Get speed of motor
 * @return speed: speed of motor in RPM
 */
float MotorEncoder::getSpeed() {
    unsigned long currentTime = millis();
    float speed;

    if (currentTime - this->lastTime > 100) {
        speed = (this->count - this->lastCount) / (float)this->countPerRev * 60;
        this->lastCount = this->count;
        this->lastTime = currentTime;
    }

    return speed;
}

/** Get distance travelled by motor
 * @return distance: distance travelled by motor
 */
float MotorEncoder::getDistance() {
    return this->count / this->countPerRev * PI * this->wheelDiameter;
}

/** Update encoder count
 * attach this function to interrupt
*/
void MotorEncoder::update() {
    this->curretState = digitalRead(this->pinA) << 2 | digitalRead(this->pinB);
    // Serial.print(this->curretState); Serial.print("\t");
    // Serial.print(this->lastState); Serial.println("\t");
    if (this->curretState == 0b100 && this->lastState == 0b01) {
        this->count++;
    } else if (this->curretState == 0b101 && this->lastState == 0b00) {
        this->count--;
    }
    this->lastState = this->curretState;
}

/** Get encoder count
 * @return count: encoder count
 */
int MotorEncoder::getCount() {
    return this->count;
}


// ===============================================
// =========== Ultrasonic Sensor Class ===========
// ===============================================

/** Constructor for Ultrasonic class
 * @param echoPin: Echo pin of ultrasonic sensor
 * @param trigPin: Trig pin of ultrasonic sensor
 */
Ultrasonic::Ultrasonic(int echoPin, int trigPin, unsigned long timeout) {
    this->echoPin = echoPin;
    this->trigPin = trigPin;
    this->timeout = timeout;
    this->waitingForEcho = false;
    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);
}

void Ultrasonic::trigger() {
    digitalWrite(this->trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(this->trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(this->trigPin, LOW);
    this->echoStartTime = micros();
    this->waitingForEcho = true;
    lastTriggerTime = millis();
}

/** Get distance from ultrasonic sensor
 * @return distance: distance from ultrasonic sensor
 */
float Ultrasonic::getDistance() {
    if (millis() - lastTriggerTime > 100) trigger();
    if (waitingForEcho) {
        if (digitalRead(echoPin) == HIGH) {
            unsigned long duration = micros() - echoStartTime;
            waitingForEcho = false;
            return (duration * 0.034) / 2;
        } else if (micros() - echoStartTime > timeout) {
            waitingForEcho = false;
            return -1;
        }
    }
    return -2;
}