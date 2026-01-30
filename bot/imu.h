#ifndef IMU_H
#define IMU_H

#include "FastIMU.h"
#include <Wire.h>

class IMU {
  public:
    static const int SDA_PIN = 0;
    static const int SCL_PIN = 1;
    static const int I2C_FREQ = 400000;
    static const uint8_t IMU_ADDRESS = 0x68;

    static constexpr double ALPHA = 0.98;

    static BMI160 imu;
    static AccelData accelData;
    static GyroData gyroData;

    static calData calib;
    static constexpr double pitchBias = 5.35; // avg measuredPitch from telemetry

    static unsigned int sampleTime;  // ms
    static unsigned long lastTime;
    static unsigned long dt;


    static double pitch;
    static double gyroPitch;
    static double accelPitch;

    static bool init(unsigned int sampleTime, bool calibrate = false);
    static bool newPitch();
    static void setCalib();
    static double readPitch();
    static double readAccelPitch();
    static double readGyroPitch();
};

#endif
