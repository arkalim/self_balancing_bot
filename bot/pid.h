#ifndef PID_H
#define PID_H

#include <Arduino.h>

enum PIDDirection {
  DIRECT,
  REVERSE
};

class PID {
public:
  PID(double* input,
      double* output,
      double* setpoint,
      double kp,
      double ki,
      double kd,
      PIDDirection direction);

  void clearIntegral();
  void scaleIntegral(double amount);
  void setOutputLimits(double min, double max);
  void setTunings(double kp, double ki, double kd);
  void setSampleTime(unsigned long sampleTime);
  bool newOutput();

private:
  double* _input;
  double* _output;
  double* _setpoint;

  double _kp;
  double _ki;
  double _kd;

  double _integral;
  double _lastInput;

  double _outMin;
  double _outMax;
  double _integralLimitRatio;

  unsigned long _sampleTime;
  unsigned long _lastTime;

  PIDDirection _direction;
};

#endif
