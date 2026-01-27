#include "led.h"
#include "control.h"
#include "radio.h"

#define DT 10 //ms

void setup() { 
    Serial.begin(115200);

    LED::init();
    Radio::init();
    Control::init(DT, DT*10);

    LED::glow(LED::GREEN);
}

void loop() {
  Control::control();

  if (Radio::hasPID()) {
    PIDMessage pid = Radio::getPID();
    Control::velocityPID.SetTunings(pid.kp, pid.ki, pid.kd);
  }
}
