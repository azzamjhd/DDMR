#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

class Encoder {
   private:
    uint8_t _pinA;
    uint8_t _pinB;
    bool _reverse;
    volatile int _count;

    void _init_pins();
   public:
    Encoder(uint8_t pinA, uint8_t pinB, bool reverse = false);
    void reset();
    void setReverse(bool reverse);
    void count_isr(void);
    int getCount();
};

#endif // ENCODER_HPP