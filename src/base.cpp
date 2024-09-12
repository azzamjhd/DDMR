#include "base.h"


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

/** Set speed of motor
 * @param speed: speed of motor -255 to 255
 */
void MotorEncoder::setSpeed(int speed) {
    if (speed > 0) {
        setDir(_FORWARD);
        analogWrite(this->pwm, speed);
    } else if (speed < 0) {
        setDir(_BACKWARD);
        analogWrite(this->pwm, -speed);
    } else {
        setDir(_STOP);
        analogWrite(this->pwm, 0);
    }
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
    unsigned long timeDiff = currentTime - this->lastTime;
    float timeDiffSec = timeDiff / 1000.0;

    float revolutions = (this->count - this->lastCount) / this->countPerRev;
    this->speed = (revolutions / timeDiffSec) * 60;

    this->lastCount = this->count;
    this->lastTime = currentTime;

    return this->speed;
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
    this->curretState = digitalRead(this->pinA) | digitalRead(this->pinB) << 1;
    if (this->curretState == 0b10 && this->lastState == 0b01) {
        this->count++;
    } else if (this->curretState == 0b11 && this->lastState == 0b00) {
        this->count--;
    }
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
Ultrasonic::Ultrasonic(int echoPin, int trigPin) {
    this->echoPin = echoPin;
    this->trigPin = trigPin;
    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);
}

/** Get distance from ultrasonic sensor
 * @return distance: distance from ultrasonic sensor
 */
int Ultrasonic::getDistance() {
    digitalWrite(this->trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(this->trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(this->trigPin, LOW);
    this->duration = pulseIn(this->echoPin, HIGH);
    this->distance = this->duration * 0.034 / 2;
    return this->distance;
}