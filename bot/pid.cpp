#include "pid.h"

PID::PID(double* input,
         double* output,
         double* setpoint,
         double kp,
         double ki,
         double kd,
         PIDDirection direction)
  : _input(input),
    _output(output),
    _setpoint(setpoint),
    _integral(0),
    _lastInput(0),
    _outMin(-255),
    _outMax(255),
    _sampleTime(10),
    _lastTime(0),
    _direction(direction),
    _integralLimitRatio(0.2),
    _expo(0)
{
  setTunings(kp, ki, kd);
}

void PID::setTunings(double kp, double ki, double kd) {
  if (kp < 0 || ki < 0 || kd < 0) return;

  double sampleTimeSec = (double)_sampleTime / 1000.0;

  _kp = kp;
  _ki = ki * sampleTimeSec;
  _kd = kd / sampleTimeSec;

  if (_direction == REVERSE) {
    _kp = -_kp;
    _ki = -_ki;
    _kd = -_kd;
  }
}

void PID::setExpo(double expo) { _expo = expo; }

void PID::setSampleTime(unsigned long sampleTime) {
  if (sampleTime == 0) return;

  double ratio = (double)sampleTime / (double)_sampleTime;
  _ki *= ratio;
  _kd /= ratio;
  _sampleTime = sampleTime;
}

void PID::setOutputLimits(double min, double max) {
  if (min >= max) return;

  _outMin = min;
  _outMax = max;

  if (*_output > _outMax) *_output = _outMax;
  else if (*_output < _outMin) *_output = _outMin;

  if (_integral > _outMax) _integral = _outMax;
  else if (_integral < _outMin) _integral = _outMin;
}

void PID::clearIntegral() {
  _integral = 0;
}

void PID::scaleIntegral(double amount) {
  _integral *= amount;
}

bool PID::newOutput() {
  unsigned long now = millis();
  unsigned long dt = now - _lastTime;

  if (dt < _sampleTime) {
    return false;
  }

  double input = *_input;
  double error = *_setpoint - input;
  double dInput = input - _lastInput;

  // Expo
  double scale = exp(_expo * error);
  scale = constrain(scale, 1.0, 4.0);

  // Integral
  _integral += _ki * error;
  double integralLimit = (_outMax - _outMin) * 0.5 * _integralLimitRatio;
  if (_integral > integralLimit) _integral = integralLimit;
  else if (_integral < -integralLimit) _integral = -integralLimit;

  // PID output
  double output = scale * _kp * error + _integral - _kd * dInput;

  if (output > _outMax) output = _outMax;
  else if (output < _outMin) output = _outMin;

  *_output = output;

  _lastInput = input;
  _lastTime = now;

  return true;
}
