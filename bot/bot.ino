#include "imu.h"
#include "motors.h"
#include "led.h"
#include "control.h"
#include "radio.h"

#define DT 10 //ms

void setup() { 
    Serial.begin(115200);

    LED::init();
    IMU::init(DT);
    Motors::init(DT*10);
    Control::init(DT, DT*10);
    Radio::init();

    LED::glow(LED::GREEN);
}

void loop() {
  if (IMU::newPitch()) { Control::measuredPitch = IMU::readPitch(); }

  if (Control::fallen()) {
    Motors::move(0);
    LED::glow(LED::RED);
    return;
  }

  if (Control::newPWM()) { Motors::move((int)Control::pwm); }

  if (Motors::newVelocity()) { Control::measuredVelocity = Motors::readVelocity(); }

  Control::newTargetPitch();

  if (Radio::hasControl()) {
    ControlMessage control = Radio::getControl();
    Control::move(control.move);
  }
}
