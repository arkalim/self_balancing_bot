#ifndef CONTROL_H
#define CONTROL_H

#include <PID_v1.h>

class Control {
  public:
    static double measuredPitch;
    static double targetPitch;
    static double pwm;

    static double measuredVelocity;
    static double targetVelocity;

    static PID pitchPID;
    static PID velocityPID;

    static void init(unsigned int pitchSampleTime, unsigned int velocitySampleTime);
    static bool newPWM();
    static bool newTargetPitch();
    static bool fallen();
};

#endif
