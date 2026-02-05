#include <Arduino.h>
#include "led.h"
#include "control.h"
#include "radio.h"

void setup() { 
  // Serial.begin(115200);

  LED::init();
  Radio::init();
  Control::init(true);

  LED::glow(LED::MAGENTA);
}

void loop() {
  Control::loop();

  if (Control::fallen()) { 
    LED::glow(LED::RED); 
  } else {
    LED::glow(LED::GREEN);
  }

  if (Radio::hasControl()) { Control::move(Radio::getControl()); }

  if (Radio::hasPID()) { Control::tunePID(Radio::getPID()); }

  if (Control::newTelemetry()) { Radio::sendTelemetry(Control::readTelemetry()); }
}
