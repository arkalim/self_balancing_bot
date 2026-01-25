#include "radio.h"

struct PIDMessage {
    double Kp;
    double Ki;
    double Kd;
};

PIDMessage pid;

void setup() {
    Serial.begin(115200);
    Radio::init();
}

void loop() {
    if (Radio::readFromSerial(pid)) {
        Radio::send(Radio::peerMAC, pid);
    }

    if (Radio::hasNewMessage()) {
        PIDMessage rx = Radio::getMessage<PIDMessage>();
        Serial.printf("%.2f %.2f %.2f\n", rx.Kp, rx.Ki, rx.Kd);
    }
}

// #include <WiFi.h>

// void setup() {
//   Serial.begin(115200);
//   WiFi.mode(WIFI_STA);
//   Serial.println(WiFi.macAddress());
// }

// void loop() {}

// 20:6E:F1:6E:08:80
