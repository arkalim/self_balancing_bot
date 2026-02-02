#include "led.h"
#include "radio.h"
#include "serial_handler.h"
#include "joystick.h"

void setup() {
    Serial.begin(115200);

    LED::init();
    Radio::init();
    SerialHandler::init();
    Joystick::init();

    LED::glow(LED::CYAN);
}

void loop() {
    SerialHandler::refresh();

    if (SerialHandler::hasControlMessage()) { Radio::sendControl(SerialHandler::readControlMessage()); }
    if (SerialHandler::hasPIDMessage()) { Radio::sendPID(SerialHandler::readPIDMessage()); }
    if (Radio::hasTelemetry()) { SerialHandler::writeTelemetryMessage(Radio::readTelemetry()); }

    if (Joystick::newControl()) { Radio::sendControl(Joystick::readControl()); }
}