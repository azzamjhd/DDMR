#include "encoder.hpp"

Encoder::Encoder(uint8_t pinA, uint8_t pinB, bool reverse) {
  _pinA = pinA;
  _pinB = pinB;
  _count = 0;
  _reverse = reverse;
  _init_pins();
}

void Encoder::_init_pins() {
  pinMode(_pinA, INPUT);
  pinMode(_pinB, INPUT);
}

void Encoder::reset() {
  _count = 0;
}

void Encoder::setReverse(bool reverse) {
  _reverse = reverse;
}

int32_t Encoder::getCount() {
  int32_t count;
  { count = _count; }
  return _reverse ? -count : count;
}

void Encoder::count_isr(void) {
  if (digitalRead(_pinB) == HIGH) {
    _count--;
  } else {
    _count++;
  }
}

void Encoder::printCount(void) const {
  Serial.print(_count);
}