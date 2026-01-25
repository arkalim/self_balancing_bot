#include "transmitter.h"

void setup() {
    Serial.begin(115200);
    Transmitter::init();
    Serial.println("Enter Kp Ki Kd separated by spaces, e.g., 1.0 0.5 0.1\n");
}

void loop() {
    if (Transmitter::readFromSerial()) { Transmitter::send(); }
}
