#include "base.h"

// ===============================================
// =========== Ultrasonic Sensor Class ===========
// ===============================================

/** Constructor for Ultrasonic class
 * @param echoPin: Echo pin of ultrasonic sensor
 * @param trigPin: Trig pin of ultrasonic sensor
 */
Ultrasonic::Ultrasonic(int echoPin, int trigPin, unsigned long timeout) {
    _echoPin = echoPin;
    _trigPin = trigPin;
    _timeout = timeout;
    _waitingForEcho = false;
    pinMode(_echoPin, INPUT);
    pinMode(_trigPin, OUTPUT);
}

void Ultrasonic::trigger() {
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
    _echoStartTime = micros();
    _waitingForEcho = true;
    _lastTriggerTime = millis();
}

/** Get distance from ultrasonic sensor
 * @return distance: distance from ultrasonic sensor
 */
float Ultrasonic::getDistance() {
    if (millis() - _lastTriggerTime > 100) trigger();
    if (_waitingForEcho) {
        if (digitalRead(_echoPin) == HIGH) {
            unsigned long duration = micros() - _echoStartTime;
            _waitingForEcho = false;
            return (duration * 0.034) / 2;
        } else if (micros() - _echoStartTime > _timeout) {
            _waitingForEcho = false;
            return -1;
        }
    }
    return -2;
}