#include "joystick.h"

unsigned long Joystick::lastTime;
int Joystick::lastMove = 0;
int Joystick::lastTurn = 0;

ControlMessage Joystick::controlMessage;

void Joystick::init() {
  analogReadResolution(8);          // 0–4095
  analogSetAttenuation(ADC_11db);    // Full range ~0–3.3V

  lastTime = millis();
  Serial.println("Joystick Ready");
}

bool Joystick::newControl() {
  unsigned long now = millis();
  unsigned long dt = now - lastTime;
  if (dt < sampleTime) { return false; }
  lastTime = now;

  int moveValue = analogRead(MOVE_PIN);
  int turnValue = analogRead(TURN_PIN);

  // Serial.printf("%i %i\n", moveValue, turnValue);

  int newMove = getControlValue(moveValue, MOVE_CENTER, MOVE_REVERSED);
  int newTurn = getControlValue(turnValue, TURN_CENTER, TURN_REVERSED);

  // Only update if values change
  if (newMove != lastMove || newTurn != lastTurn) {
    lastMove = newMove;
    lastTurn = newTurn;

    controlMessage.move = newMove;
    controlMessage.turn = newTurn;
    return true;
  }

  return false;
}

int Joystick::getControlValue(int rawValue, int centerValue, bool reversed) {
  if (abs(rawValue - centerValue) < DEADZONE) {
    return 0;
  }

  int controlValue = 0;
  if (rawValue > centerValue) {
    controlValue = map(rawValue, centerValue + DEADZONE, 255, 0, ControlMessage::MAX);
  } else {
    controlValue = map(rawValue, 0, centerValue - DEADZONE, -ControlMessage::MAX, 0);
  }

  if (reversed) {
    return -controlValue;
  }
  return controlValue;
}

ControlMessage Joystick::readControl() {
  return controlMessage;
}