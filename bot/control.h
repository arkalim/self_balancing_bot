#ifndef CONTROL_H
#define CONTROL_H

#include <PID_v1.h>
#include "messages.h"
#include "imu.h"
#include "motors.h"

class Control {
  public:
    static const unsigned int pitchSampleTime = 10;       //ms
    static const unsigned int velocitySampleTime = 100;   //ms
    static const unsigned int telemetrySampleTime = 100;  //ms

    static bool enableTelemetry;
    static const unsigned int fallPitch = 45;

    static double measuredPitch;
    static double targetPitch;

    static double measuredVelocity;
    static double targetVelocity;

    static double measuredVelocityDiff;
    static double targetVelocityDiff;

    static double pwm;
    static double pwmDiff;

    static constexpr double velocityLimit = 1.0;
    static constexpr double velocityDiffLimit = 0.5;

    static PID pitchPID;
    static PID velocityPID;
    static PID velocityDiffPID;

    static void init(bool enableTelemetry = false);
    static void loop();
    static bool newPWM();
    static bool newTargetPitch();
    static bool newPWMDiff();

    static void tunePID(PIDMessage pidMessage);
    static void move(ControlMessage controlMessage);

    static unsigned long lastTelemetryTime;
    static bool newTelemetry();
    static TelemetryMessage readTelemetry();
};

#endif
