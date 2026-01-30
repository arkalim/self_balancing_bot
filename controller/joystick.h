#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>
#include "messages.h"

class Joystick {
public:
  static void init();

  static bool newControl();

  static ControlMessage readControl();
  static int getControlValue(int rawValue, int centerValue);

private:
  static const unsigned int sampleTime = 100; // ms
  static unsigned long lastTime;
  static int lastMove;
  static int lastTurn;

  static const int MOVE_PIN = 0;
  static const int TURN_PIN = 2;

  static const int DEADZONE = 3;
  static const int MOVE_CENTER = 145;
  static const int TURN_CENTER = 147;

  static ControlMessage controlMessage;

  static bool _hasControlMessage;
};

#endif
