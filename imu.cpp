#include "imu.h"

bool IMU::init(int sampleTime) {
    this->sampleTime = sampleTime;

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ);

    int err = imu.init(calib, IMU_ADDRESS);
    if (err != 0) {
        Serial.print("Error initializing IMU: ");
        Serial.println(err);
        return false;
    }

    // initialize pitch from accelerometer at the start
    imu.update();
    imu.getAccel(&accelData);
    gyroPitch = readAccelPitch();
    pitch = gyroPitch;

    lastTime = millis();
    return true;
}

bool IMU::compute() {
    // compute pitch only if sampleTime has passed
    unsigned long now = millis();
    dt = (now - lastTime);
    if (dt < sampleTime) {return false;}
    lastTime = now;

    imu.update();
    imu.getAccel(&accelData);
    imu.getGyro(&gyroData);

    // Complementary filter
    pitch = ALPHA * readGyroPitch() + (1.0 - ALPHA) * readAccelPitch();

    // Correct for gyro drift
    gyroPitch = pitch;

    return true;
}

float IMU::readPitch() {
    return pitch;
}

float IMU::readAccelPitch() {
    return atan2(-accelData.accelY, accelData.accelZ) * 180.0 / PI;
}

float IMU::readGyroPitch() {
    gyroPitch = gyroPitch - gyroData.gyroX * (dt / 1000.0); // gyro in deg/sec
    return gyroPitch;
}