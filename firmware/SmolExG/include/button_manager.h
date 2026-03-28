#ifndef BUTTON_MANAGER_H
#define BUTTON_MANAGER_H

#include <Arduino.h>

class ButtonManager {
  public:
    enum class State {
      NONE,  // No button press detected.
      SHORT,  // Short button press detected (released).
      LONG  // Long button press detected (still held).
    };

    ButtonManager(int pin) : pin_(pin), keydown_time_ms_(-1) {}
    State Update() {
      bool pressed = digitalRead(pin_) == LOW; // We assume active low.
      State state = State::NONE;
      if (pressed) {
        if (keydown_time_ms_  == -1) {
          // We have a new press! We don't know if it's going to be a short or long press yet.
          keydown_time_ms_ = millis();
          return State::NONE;
        } else {
          // We are continuing to hold down the key. See if we have reached the long press threshold.
          int32_t elapsed_time = millis() - keydown_time_ms_;
          if (elapsed_time >= LONG_PRESS_THRESHOLD_MS) {
            return State::LONG;
          } else {
            return State::NONE;
          }
        }        
      } else {
        // Have we just released it?
        if (keydown_time_ms_ != -1) {
          int32_t elapsed_time = millis() - keydown_time_ms_;
          if (elapsed_time >= SHORT_PRESS_THRESHOLD_MS && elapsed_time < LONG_PRESS_THRESHOLD_MS) {
            state = State::SHORT;
          } else {
            // Otherwise we are releasing either a long press, or a bounce.
            state = State::NONE;
          }
          keydown_time_ms_ = -1;
        }
      }
        
      return state;
    }

    int32_t LongPressDuration() {
      return millis() - keydown_time_ms_;
    }
  
  private:
    static const int SHORT_PRESS_THRESHOLD_MS = 30; // Presses shorter than this will not be registered.
    static const int LONG_PRESS_THRESHOLD_MS = 500; // Presses longer than this will register as long press (press and hold).

    int pin_;
    int32_t keydown_time_ms_;  // Either -1 if the button is not currently pressed, or when it was pressed.
};

#endif // BUTTON_MANAGER_H
