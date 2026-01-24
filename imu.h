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

    static constexpr float ALPHA = 0.98;  // Complementary filter coefficient

    BMI160 imu;
    AccelData accelData;
    GyroData gyroData;

    calData calib = {0};

    int sampleTime = 10; // ms

    float pitch = 0;
    float gyroPitch = 0;
    float accelPitch = 0;

    bool init(int sampleTime);
    bool compute();
    float readPitch();
    float readAccelPitch();
    float readGyroPitch();
};

#endif
