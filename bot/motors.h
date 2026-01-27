#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

class Motors {
  public:
    static void init(unsigned int sampleTime, bool power = true);
    static const int ENC_PULSE_PER_REV = 7 * 30;  // 7 ppr for motor × 30 (gear ratio)

    static unsigned int sampleTime; // ms
    static unsigned long lastTime;
    static unsigned long dt;

    static const int LEFT_PWM_PIN = 2;
    static const int LEFT_DIR1_PIN = 21;
    static const int LEFT_DIR2_PIN = 20;
    static const int LEFT_ENC_PIN = 5;

    static const int RIGHT_PWM_PIN = 6;
    static const int RIGHT_DIR1_PIN = 7;
    static const int RIGHT_DIR2_PIN = 4;
    static const int RIGHT_ENC_PIN = 3;

    static const int PWM_FREQ = 20000;
    static const int PWM_RES = 8;
    static bool power;

    static int leftDir;
    static volatile long leftEncoderCount;
    static long lastLeftEncoderCount;
    static double leftVelocity;

    static int rightDir;
    static volatile long rightEncoderCount;
    static long lastRightEncoderCount;
    static double rightVelocity;

    static void setLeftMotor(int speed);
    static void setRightMotor(int speed);
    static void move(int speed);
    static void turn(int speed);
    static void stop();
    static bool newVelocity();
    static double readVelocity();
    static double readLeftVelocity();
    static double readRightVelocity();
};

#endif
