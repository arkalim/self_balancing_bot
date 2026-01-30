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

  int newMove = getControlValue(moveValue, MOVE_CENTER);
  int newTurn = getControlValue(turnValue, TURN_CENTER);

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

int Joystick::getControlValue(int rawValue, int centerValue) {
  if (abs(rawValue - centerValue) < DEADZONE) {
    return 0;
  }

  if (rawValue > centerValue) {
    return map(rawValue, centerValue + DEADZONE, 255, 0, ControlMessage::MAX);
  } else {
    return map(rawValue, 0, centerValue - DEADZONE, -ControlMessage::MAX, 0);
  }
}

ControlMessage Joystick::readControl() {
  return controlMessage;
}