#include <Arduino.h>
#include <PID_v1.h>

#include "imu.h"
#include "motors.h"
#include "led.h"

#define DT 10 //ms

double measuredPitch = 0.0;
double targetPitch = 0.0;
double pwm = 0.0;

double targetVelocity = 0.0;     // rpm
double measuredVelocity = 0.0;

IMU imu;

PID balancePID(&measuredPitch, &pwm, &targetPitch, 25.0, 0.0, 0.5, DIRECT);
PID velocityPID(&measuredVelocity, &targetPitch, &targetVelocity, 5, 0.0, 0.0, REVERSE);

void setup() { 
    LED::init();

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
    velocityPID.SetSampleTime(DT*100);

    LED::glow(LED::GREEN);
}

void loop() {
    balancePID.SetMode(AUTOMATIC);
    if (balancePID.Compute()) { 
      measuredPitch = imu.readPitch();

      // Safety cutoff: bot has fallen
      if (abs(measuredPitch) > 30.0) {
          balancePID.SetMode(MANUAL);   // stop PID integrating
          pwm = 0;
      }
      Motors::move((int)pwm);
    }

    // Velocity PID
    if (velocityPID.Compute()) { 
      measuredVelocity = Motors::readVelocity(DT*100);
      if (measuredVelocity > 0) {
        LED::glow(LED::BLUE);
      } else {
        LED::glow(LED::RED);
      }
    }
}
