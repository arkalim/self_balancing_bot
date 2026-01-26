#ifndef CONTROL_H
#define CONTROL_H

#include <PID_v1.h>
#include "messages.h"

class Control {
  public:
    static double measuredPitch;
    static double targetPitch;
    static double pwm;

    static double measuredVelocity;
    static double targetVelocity;
    static constexpr double moveVelocityLimit = 1.0;

    static PID pitchPID;
    static PID velocityPID;

    static void init(unsigned int pitchSampleTime, unsigned int velocitySampleTime);
    static bool newPWM();
    static bool newTargetPitch();
    static bool fallen();
    static void move(int velocity);
    static void turn();
    static TelemetryMessage readTelemetry();
};

#endif
