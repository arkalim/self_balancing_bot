#include <Arduino.h>
#include "radio.h"

void setup() {
    Serial.begin(115200);
    delay(500);

    Radio::init();
    Serial.println("Enter PID as: kp ki kd");
    Serial.println("Example: 25.0 1.2 0.8");
}

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) return;

        double kp, ki, kd;
        int parsed = sscanf(line.c_str(), "%lf %lf %lf", &kp, &ki, &kd);

        if (parsed == 3) {
            PIDMessage pid{};
            pid.type = MessageType::PID;
            pid.kp = kp;
            pid.ki = ki;
            pid.kd = kd;

            Radio::sendPID(pid);

            Serial.print("Sent PID -> ");
            Serial.print("Kp: "); Serial.print(kp);
            Serial.print(" Ki: "); Serial.print(ki);
            Serial.print(" Kd: "); Serial.println(kd);
        } else {
            Serial.println("Invalid format. Use: kp ki kd");
        }
    }

    delay(10);
}