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

PID balancePID(&measuredPitch, &pwm, &targetPitch, 35.0, 0.1, 0.6, DIRECT);
PID velocityPID(&measuredVelocity, &targetPitch, &targetVelocity, 5, 1, 0.1, REVERSE);

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
    velocityPID.SetOutputLimits(-15, 15);
    velocityPID.SetSampleTime(DT*10);

    LED::glow(LED::GREEN);
}

void loop() {
  if (IMU::ready()) { measuredPitch = IMU::readPitch(); }

  if (abs(measuredPitch) > 30.0) {
    Motors::move(0);
    LED::glow(LED::RED);
    return;
  }

  if (balancePID.Compute()) { Motors::move((int)pwm); }

  if (Motors::ready()) { measuredVelocity = Motors::readVelocity(); }

  velocityPID.Compute();

  if (Receiver::hasNewMessage()) {
    Receiver::Message message = Receiver::getMessage();
    balancePID.SetTunings(message.Kp, message.Ki, message.Kd);
  }
  Serial.printf("Measured Pitch: %.4f | Target Pitch: %.4f | PWM: %.2f | Measured Velocity: %.2f \n", measuredPitch, targetPitch, pwm, measuredVelocity);
}
