#include "control.h"

double Control::measuredPitch = 0;
double Control::targetPitch = 0;

double Control::measuredVelocity = 0;
double Control::targetVelocity = 0;
double Control::desiredTargetVelocity = 0;

double Control::measuredVelocityDiff = 0;
double Control::targetVelocityDiff = 0;

double Control::pwm = 0;
double Control::pwmDiff = 0;

bool Control::enableTelemetry;
unsigned long Control::lastTelemetryTime;
ControlMessage Control::controlMessage = { MessageType::CONTROL, 0, 0 };

PID Control::pitchPID(&Control::measuredPitch, &Control::pwm, &Control::targetPitch, 35, 1, 1, DIRECT);
PID Control::velocityPID(&Control::measuredVelocity, &Control::targetPitch, &Control::targetVelocity, 3, 0.001, 0.003, REVERSE);
PID Control::velocityDiffPID(&Control::measuredVelocityDiff, &Control::pwmDiff, &Control::targetVelocityDiff, 30, 0, 0.25, DIRECT);

void Control::init(bool enableTelemetry) {
  Control::enableTelemetry = enableTelemetry;
  lastTelemetryTime = millis();

  pitchPID.setOutputLimits(-255, 255);
  pitchPID.setSampleTime(pitchSampleTime);

  velocityPID.setOutputLimits(-15, 15);
  velocityPID.setSampleTime(velocitySampleTime);

  velocityDiffPID.setOutputLimits(-100, 100);
  velocityDiffPID.setSampleTime(velocitySampleTime);

  IMU::init(pitchSampleTime);
  Motors::init(velocitySampleTime);
}

bool Control::fallen() { return abs(measuredPitch) > fallPitch; }

bool Control::newPWM() { return pitchPID.newOutput(); }

bool Control::newTargetPitch() { return velocityPID.newOutput(); }

bool Control::newPWMDiff() { return velocityDiffPID.newOutput(); }

void Control::loop() {
  if (IMU::newPitch()) { measuredPitch = IMU::readPitch(); }

  if (fallen()) { 
    Motors::move(0, 0); 
    pitchPID.clearIntegral();
    velocityPID.clearIntegral();
    velocityDiffPID.clearIntegral();
    return; 
  }

  if (newPWM()) {
    Motors::move((int)pwm, (int)pwmDiff);
  }

  if (Motors::newVelocity()) {
    measuredVelocity = Motors::readVelocity();
    measuredVelocityDiff = Motors::readVelocityDiff();
  }

  if (newTargetPitch()) {
    setTargetVelocity();
  }

  newPWMDiff();
}

void Control::setTargetVelocity() {
  double dv = desiredTargetVelocity - targetVelocity;
  bool accelerating = (abs(desiredTargetVelocity) > abs(targetVelocity));
  double acceleration = accelerating ? MAX_ACCELERATION : MAX_DECELERATION;
  double dvLimit = acceleration * velocitySampleTime / 1000.0;
  dv = constrain(dv, -dvLimit, dvLimit);
  targetVelocity += dv;
}

void Control::move(ControlMessage newControlMessage) {

  // clear pitch and velocity integral on move direction change
  if (controlMessage.move * newControlMessage.move <= 0) {
    pitchPID.clearIntegral();
    velocityPID.clearIntegral();

  }

  // clear velocity diff integral on turn direction change
  if (controlMessage.turn * newControlMessage.turn <= 0) {
    velocityDiffPID.clearIntegral();
  }

  controlMessage = newControlMessage;

  int moveInput = constrain(controlMessage.move, -ControlMessage::MAX, ControlMessage::MAX);
  int turnInput = constrain(controlMessage.turn, -ControlMessage::MAX, ControlMessage::MAX);
  desiredTargetVelocity = velocityLimit * moveInput / ControlMessage::MAX;
  targetVelocityDiff = velocityDiffLimit * turnInput / ControlMessage::MAX;
}

void Control::tunePID(PIDMessage pidMessage) {
  switch(pidMessage.pidType) {
    case PIDType::PITCH:
    pitchPID.setTunings(pidMessage.kp, pidMessage.ki, pidMessage.kd);
    break;

    case PIDType::VELOCITY:
    velocityPID.setTunings(pidMessage.kp, pidMessage.ki, pidMessage.kd);
    break;

    case PIDType::VELOCITY_DIFF:
    velocityDiffPID.setTunings(pidMessage.kp, pidMessage.ki, pidMessage.kd);
    break;
  }
}

bool Control::newTelemetry() {
  if (!enableTelemetry) { return false; }
  unsigned long now = millis();
  unsigned long dt = now - lastTelemetryTime;
  if (dt > telemetrySampleTime) {
    lastTelemetryTime = now;
    return true;
  }
  return false;

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
