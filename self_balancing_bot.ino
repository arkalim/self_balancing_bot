#include <Arduino.h>
#include <PID_v1.h>

#include "imu.h"
#include "motors.h"

#define DT 10 //ms

double measuredPitch = 0.0;
double targetPitch = 0.0;
double pwm = 0.0;

double targetVelocity = 0.0;     // rpm
double measuredVelocity = 0.0;

IMU imu;

PID balancePID(&measuredPitch, &pwm, &targetPitch, 25.0, 0.0, 0.5, DIRECT);
PID velocityPID(&measuredVelocity, &targetPitch, &targetVelocity, 1.0, 0.0, 0.0, REVERSE);

void setup() { 
    // wait for IMU to start
    if (!imu.init(DT)) {
        while (true) {;}
    }

    Motors::init();

    balancePID.SetMode(AUTOMATIC);
    balancePID.SetOutputLimits(-200, 200);
    balancePID.SetSampleTime(DT);

    velocityPID.SetMode(AUTOMATIC);
    velocityPID.SetOutputLimits(-5, 5);
    velocityPID.SetSampleTime(DT*5);
}

void loop() {
    if(imu.compute()){ measuredPitch = imu.readPitch(); }

    // Safety cutoff: bot has fallen
    if (abs(measuredPitch) > 30.0) {
        pwm = 0;
        balancePID.SetMode(MANUAL);   // stop PID integrating
        Motors::move(pwm);
        return;
    }

    // Normal operation
    balancePID.SetMode(AUTOMATIC);
    if (balancePID.Compute()) { Motors::move((int)pwm); }

    // Velocity PID
    if (velocityPID.Compute()) { measuredVelocity = Motors::readVelocity(DT*5); }
}
