#include <Arduino.h>
#include "control.h"

double Control::measuredPitch = 0;
double Control::targetPitch = 0;

double Control::measuredVelocity = 0;
double Control::targetVelocity = 0;

double Control::measuredVelocityDiff = 0;
double Control::targetVelocityDiff = 0;

double Control::pwm = 0;
double Control::pwmDiff = 0;

bool Control::enableTelemetry;
unsigned long Control::lastTelemetryTime;

PID Control::pitchPID(&Control::measuredPitch, &Control::pwm, &Control::targetPitch, 35.0, 0.1, 0.6, DIRECT);
PID Control::velocityPID(&Control::measuredVelocity, &Control::targetPitch, &Control::targetVelocity, 4.5, 0.5, 0.185, REVERSE);
PID Control::velocityDiffPID(&Control::measuredVelocityDiff, &Control::pwmDiff, &Control::targetVelocityDiff, 50, 5, 0, DIRECT);

void Control::init(bool enableTelemetry) {
  Control::enableTelemetry = enableTelemetry;

  IMU::init(pitchSampleTime);
  Motors::init(velocitySampleTime);
  lastTelemetryTime = millis();

  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-255, 255);
  pitchPID.SetSampleTime(pitchSampleTime);

  velocityPID.SetMode(AUTOMATIC);
  velocityPID.SetOutputLimits(-30, 30);
  velocityPID.SetSampleTime(velocitySampleTime);

  velocityDiffPID.SetMode(AUTOMATIC);
  velocityDiffPID.SetOutputLimits(-30, 30);
  velocityDiffPID.SetSampleTime(velocitySampleTime);
}

bool Control::newPWM() { return pitchPID.Compute(); }

bool Control::newTargetPitch() { return velocityPID.Compute(); }

bool Control::newPWMDiff() { return velocityDiffPID.Compute(); }

void Control::loop() {
  if (IMU::newPitch()) { measuredPitch = IMU::readPitch(); }

  if (abs(measuredPitch) > fallPitch) { Motors::move(0, 0); return; }

  if (newPWM()) { Motors::move((int)pwm, (int)pwmDiff); }

  if (Motors::newVelocity()) {
    measuredVelocity = Motors::readVelocity();
    measuredVelocityDiff = Motors::readVelocityDiff();
  }

  newTargetPitch();
  newPWMDiff();
}

void Control::move(ControlMessage controlMessage) {
  int moveInput = constrain(controlMessage.move, -100, 100);
  int turnInput = constrain(controlMessage.turn, -100, 100);
  targetVelocity = velocityLimit * moveInput / 100.0;
  targetVelocityDiff = velocityDiffLimit * turnInput / 100.0;
}

void Control::tunePID(PIDMessage pidMessage) {
  switch(pidMessage.pidType) {
    case PIDType::PITCH:
    pitchPID.SetTunings(pidMessage.kp, pidMessage.ki, pidMessage.kd);
    break;

    case PIDType::VELOCITY:
    velocityPID.SetTunings(pidMessage.kp, pidMessage.ki, pidMessage.kd);
    break;

    case PIDType::VELOCITY_DIFF:
    velocityDiffPID.SetTunings(pidMessage.kp, pidMessage.ki, pidMessage.kd);
    break;
  }
}

bool Control::newTelemetry() {
  if (!enableTelemetry) { return false; }
  unsigned long now = millis();
  unsigned long dt = now - lastTelemetryTime;
  lastTelemetryTime = now;
  return dt > telemetrySampleTime;
}

TelemetryMessage Control::readTelemetry() {
  TelemetryMessage message;
  message.targetPitch = targetPitch;
  message.measuredPitch = measuredPitch;
  message.targetVelocity = targetVelocity;
  message.measuredVelocity = measuredVelocity;
  message.targetVelocityDiff = targetVelocityDiff;
  message.measuredVelocityDiff = measuredVelocityDiff;
  message.pwm = static_cast<int>(pwm);
  message.pwmDiff = static_cast<int>(pwmDiff);
  return message;
}
