#include <Arduino.h>
#include "control.h"

double Control::measuredPitch = 0;
double Control::targetPitch = 0;
double Control::pwm = 0;

double Control::measuredVelocity = 0;
double Control::targetVelocity = 0;

unsigned int fallPitch = 30;

PID Control::pitchPID(&Control::measuredPitch, &Control::pwm, &Control::targetPitch, 35.0, 0.1, 0.6, DIRECT);
PID Control::velocityPID(&Control::measuredVelocity, &Control::targetPitch, &Control::targetVelocity, 5, 1, 0.1, REVERSE);

void Control::init(unsigned int pitchSampleTime, unsigned int velocitySampleTime) {
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-255, 255);
  pitchPID.SetSampleTime(pitchSampleTime);

  velocityPID.SetMode(AUTOMATIC);
  velocityPID.SetOutputLimits(-15, 15);
  velocityPID.SetSampleTime(velocitySampleTime);
}

bool Control::newPWM() { return pitchPID.Compute(); }

bool Control::newTargetPitch() { return velocityPID.Compute(); }

bool Control::fallen() { return (abs(measuredPitch) > fallPitch); }

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
