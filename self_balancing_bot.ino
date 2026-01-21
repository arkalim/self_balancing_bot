#include <Arduino.h>
#include "imu.h"

#define DT 10

IMU imu;

double measuredPitch = 0.0;

void setup() { 
    // wait for IMU to start
    if (!imu.init(DT)) {
        while (true) {;}
    }

    Serial.begin(115200);
    while (!Serial) {;}
}

void loop() {
    measuredPitch = imu.readPitch();
    Serial.println(measuredPitch);
}
