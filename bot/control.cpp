#include <Arduino.h>
#include "control.h"

double Control::measuredPitch = 0;
double Control::targetPitch = 0;
double Control::pwm = 0;

double Control::measuredVelocity = 0;
double Control::targetVelocity = 0;

unsigned int fallPitch = 45;

PID Control::pitchPID(&Control::measuredPitch, &Control::pwm, &Control::targetPitch, 35.0, 0.1, 0.6, DIRECT);
PID Control::velocityPID(&Control::measuredVelocity, &Control::targetPitch, &Control::targetVelocity, 4.5, 0.5, 0.185, REVERSE);

void Control::init(unsigned int pitchSampleTime, unsigned int velocitySampleTime) {
  IMU::init(pitchSampleTime);
  Motors::init(velocitySampleTime);

  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-255, 255);
  pitchPID.SetSampleTime(pitchSampleTime);

  velocityPID.SetMode(AUTOMATIC);
  velocityPID.SetOutputLimits(-30, 30);
  velocityPID.SetSampleTime(velocitySampleTime);
}

bool Control::newPWM() { return pitchPID.Compute(); }

bool Control::newTargetPitch() { return velocityPID.Compute(); }

void Control::control() {
  if (IMU::newPitch()) { measuredPitch = IMU::readPitch(); }

  if (abs(measuredPitch) > fallPitch) { Motors::move(0); return; }

  if (newPWM()) { Motors::move((int)pwm); }

  if (Motors::newVelocity()) { measuredVelocity = Motors::readVelocity(); }

  newTargetPitch();
}

void Control::move(int percent) {
  percent = constrain(percent, -100, 100);
  targetVelocity = moveVelocityLimit * percent / 100.0;
}

TelemetryMessage Control::readTelemetry() {
  TelemetryMessage message;
  message.targetPitch = targetPitch;
  message.measuredPitch = measuredPitch;
  message.targetVelocity = targetVelocity;
  message.measuredVelocity = measuredVelocity;
  message.pwm = static_cast<int>(pwm);
  return message;
}
