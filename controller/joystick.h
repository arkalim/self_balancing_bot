#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>
#include "messages.h"

class Joystick {
public:
  static void init();

  static bool newControl();

  static ControlMessage readControl();
  static int getControlValue(int rawValue, int centerValue, bool reversed);

private:
  static const unsigned int sampleTime = 100; // ms
  static unsigned long lastTime;
  static int lastMove;
  static int lastTurn;

  static const int MOVE_PIN = 1;
  static const int TURN_PIN = 0;

  static const int DEADZONE = 3;
  static const int MOVE_CENTER = 137;
  static const bool MOVE_REVERSED = true;
  static const int TURN_CENTER = 140;
  static const bool TURN_REVERSED = false;

  static ControlMessage controlMessage;

  static bool _hasControlMessage;
};

#endif
