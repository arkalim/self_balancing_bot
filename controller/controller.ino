#include "led.h"
#include "radio.h"
#include "serial_handler.h"

void setup() {
    LED::init();
    Radio::init();
    SerialHandler::init();

    LED::glow(LED::BLUE);
}

void loop() {
    SerialHandler::refresh();

    if (SerialHandler::hasControlMessage()) { Radio::sendControl(SerialHandler::readControlMessage()); }
    if (SerialHandler::hasPIDMessage()) { Radio::sendPID(SerialHandler::readPIDMessage()); }
    if (Radio::hasTelemetry()) { SerialHandler::writeTelemetryMessage(Radio::readTelemetry()); }
}