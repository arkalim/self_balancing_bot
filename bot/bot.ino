#include "imu.h"
#include "motors.h"
#include "led.h"
#include "control.h"
// #include "receiver.h"

#define DT 10 //ms

static bool fallen = false;

void setup() { 
    Serial.begin(115200);

    LED::init();
    IMU::init(DT);
    Motors::init(DT*10);
    Control::init(DT, DT*10);

    // Receiver::init();
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

  // if (Receiver::hasNewMessage()) {
  //   Receiver::Message message = Receiver::getMessage();
  //   balancePID.SetTunings(message.Kp, message.Ki, message.Kd);
  // }
  // Serial.printf("Measured Pitch: %.4f | Target Pitch: %.4f | PWM: %.2f | Measured Velocity: %.2f \n", measuredPitch, targetPitch, pwm, measuredVelocity);
}
