#include <Arduino.h>
#include <PID_v1.h>

#include "imu.h"
#include "motors.h"
#include "led.h"
#include "receiver.h"

#define DT 10 //ms

double measuredPitch = 0.0;
double targetPitch = 0.0;
double pwm = 0.0;

double measuredVelocity = 0.0;
double targetVelocity = 0.0;     // rpm

PID balancePID(&measuredPitch, &pwm, &targetPitch, 25.0, 0.1, 0.5, DIRECT);
PID velocityPID(&measuredVelocity, &targetPitch, &targetVelocity, 0.01, 0.005, 0, REVERSE);

static bool fallen = false;

void setup() { 
    Serial.begin(115200);

    LED::init();
    IMU::init(DT);
    Motors::init(DT*10);
    Receiver::init();

    balancePID.SetMode(AUTOMATIC);
    balancePID.SetOutputLimits(-255, 255);
    balancePID.SetSampleTime(DT);

    velocityPID.SetMode(AUTOMATIC);
    velocityPID.SetOutputLimits(-10, 10);
    velocityPID.SetSampleTime(DT*10);

    LED::glow(LED::GREEN);
}

void loop() {
  if (IMU::ready()) { measuredPitch = IMU::readPitch(); }

  if (abs(measuredPitch) > 30.0) {
    balancePID.SetMode(MANUAL);
    velocityPID.SetMode(MANUAL);
    Motors::move(0);
    fallen = true;
    LED::glow(LED::RED);
  } else if (fallen) {
    balancePID.SetMode(AUTOMATIC);
    velocityPID.SetMode(AUTOMATIC);
    fallen = false;
    LED::glow(LED::GREEN);
  }

  if (balancePID.Compute()) { Motors::move((int)pwm); }

  if (Motors::ready()) { measuredVelocity = Motors::readVelocity(); }

  velocityPID.Compute();

  if (Receiver::hasNewMessage()) {
    Receiver::Message message = Receiver::getMessage();
    velocityPID.SetTunings(message.Kp, message.Ki, message.Kd);
  }
  Serial.printf("Measured Pitch: %.4f | Target Pitch: %.4f | PWM: %.2f | Measured Velocity: %.2f \n", measuredPitch, targetPitch, pwm, measuredVelocity);
}
