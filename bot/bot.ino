#include "led.h"
#include "control.h"
#include "radio.h"

void setup() { 
    LED::init();
    Radio::init();
    Control::init(true);

    LED::glow(LED::GREEN);
}

void loop() {
  Control::loop();

  if (Radio::hasControl()) { Control::move(Radio::getControl()); }

  if (Radio::hasPID()) { Control::tunePID(Radio::getPID()); }

  if (Control::newTelemetry()) { Radio::sendTelemetry(Control::readTelemetry()); }
}
