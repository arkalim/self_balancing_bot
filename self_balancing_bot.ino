#include <Arduino.h>
#include <PID_v1.h>

#include "imu.h"
#include "motors.h"

#define DT 10

double Kp = 25.0;
double Ki = 0.0;
double Kd = 0.5;

double measuredPitch = 0.0;
double targetPitch = 0.0;
double pwm = 0.0;

IMU imu;
PID pid(&measuredPitch, &pwm, &targetPitch, Kp, Ki, Kd, DIRECT);

void setup() { 
    // wait for IMU to start
    if (!imu.init(DT)) {
        while (true) {;}
    }

    Motors::init();

    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-200, 200);
    pid.SetSampleTime(DT);
}

void loop() {
    if(imu.compute()){ measuredPitch = imu.readPitch(); }

    // Safety cutoff: robot has fallen
    if (abs(measuredPitch) > 30.0) {
        pwm = 0;
        pid.SetMode(MANUAL);   // stop PID integrating
        Motors::moveForward(0);
        return;
    }

    // Normal operation
    pid.SetMode(AUTOMATIC);
    if (pid.Compute()) {
        Motors::moveForward((int)pwm);
    }
}
