#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

class Encoder {
 private:
  uint8_t _pinA;
  uint8_t _pinB;
  bool _reverse;
  volatile int32_t _count;

  /// @brief Initialize the pins
  void _init_pins();

 public:
  /**
   * @brief Construct a new Encoder object
   *
   * @param pinA pin A
   * @param pinB pin B
   * @param reverse reverse direction flag
   */
  Encoder(uint8_t pinA, uint8_t pinB, bool reverse = false);
  /// @brief Reset the encoder
  void reset();
  /// @brief Set the reverse direction flag
  void setReverse(bool reverse);
  /// @brief Interrupt service routine for counting. attach to timer interrupt
  void count_isr(void);
  /// @brief Get the count of the encoder
  /// @return count
  int32_t getCount();
  /// @brief Print the count of the encoder
  void printCount(void) const;
};

#endif  // ENCODER_HPP