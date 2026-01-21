#ifndef IMU_H
#define IMU_H

#include "FastIMU.h"
#include <Wire.h>

class IMU {
  public:
    bool init(int dt);
    float readPitch();

  private:
    static const int SDA_PIN = 0;
    static const int SCL_PIN = 1;
    static const int I2C_FREQ = 400000;
    static const uint8_t IMU_ADDRESS = 0x68;

    static constexpr float ALPHA = 0.98;  // Complementary filter coefficient
    static constexpr float BETA = 0.85;  // Low Pass Filter coefficient

    BMI160 imu;
    AccelData accelData;
    GyroData gyroData;

    calData calib = {
      .accelBias = {0.05, -0.09, 0.06},
      .gyroBias  = {-0.08, -0.17, 0.25}
    };

    float dt = 10;

    float pitch = 0;
    float gyroPitch = 0;
    float accelPitch = 0;

    // private methods
    float readAccelPitch();
    float readGyroPitch();
};

#endif
