#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

class Motors {
  public:
    static void init();

    // High-level movement commands
    static void moveForward(int speed);
    static void moveBackward(int speed);
    static void turnLeft(int speed);
    static void turnRight(int speed);
    static void stop();

  private:
    static const int LEFT_PWM_PIN = 2;
    static const int LEFT_DIR1_PIN = 18;
    static const int LEFT_DIR2_PIN = 19;

    static const int RIGHT_PWM_PIN = 6;
    static const int RIGHT_DIR1_PIN = 7;
    static const int RIGHT_DIR2_PIN = 8;

    static const int PWM_FREQ = 20000;
    static const int PWM_RES = 8;

    // Low-level motor control
    static void setLeftMotor(int speed);
    static void setRightMotor(int speed);
};

#endif
