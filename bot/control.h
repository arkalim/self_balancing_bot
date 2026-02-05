#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "pid.h"
#include "messages.h"
#include "imu.h"
#include "motors.h"

class Control {
  public:
    static const unsigned int pitchSampleTime = 5;       //ms
    static const unsigned int velocitySampleTime = 20;   //ms
    static const unsigned int telemetrySampleTime = 100; //ms

    static bool enableTelemetry;

    static const unsigned int fallPitch = 30;

    static double measuredPitch;
    static double targetPitch;

    static double measuredVelocity;
    static double targetVelocity;
    static double desiredTargetVelocity;
    static constexpr double MAX_ACCELERATION = 3;
    static constexpr double MAX_DECELERATION = 7;

    static double measuredVelocityDiff;
    static double targetVelocityDiff;

    static double pwm;
    static double pwmDiff;

    static constexpr double velocityLimit = 2;
    static constexpr double velocityDiffLimit = 2.5;

    static PID pitchPID;
    static PID velocityPID;
    static PID velocityDiffPID;
    static ControlMessage controlMessage;

    static void init(bool enableTelemetry = false);
    static void loop();
    static bool newPWM();
    static bool newTargetPitch();
    static bool newPWMDiff();
    static bool fallen();

    static void tunePID(PIDMessage pidMessage);
    static void move(ControlMessage controlMessage);
    static void setTargetVelocity();

    static unsigned long lastTelemetryTime;
    static bool newTelemetry();
    static TelemetryMessage readTelemetry();
};

#endif
