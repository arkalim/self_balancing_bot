#include "imu.h"

BMI160 IMU::imu;
AccelData IMU::accelData;
GyroData IMU::gyroData;

unsigned int IMU::sampleTime = 0;
unsigned long IMU::lastTime = 0;
unsigned long IMU::dt = 0;

double IMU::pitch = 0.0;
double IMU::gyroPitch = 0.0;
double IMU::accelPitch = 0.0;

calData IMU::calib = {
  .accelBias = {0.000, 0.678, -0.003},
  .gyroBias = {0.000, 0.000, 0.000}
};

bool IMU::init(unsigned int sampleTime, bool calibrate) {
    IMU::sampleTime = sampleTime;
    IMU::lastTime = millis();

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ);

    if (calibrate) { setCalib(); }

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

    return true;
}

bool IMU::ready() {
    unsigned long now = millis();
    dt = now - lastTime;
    if (dt < sampleTime) { return false; }
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

double IMU::readPitch() {
    return pitch;
}

double IMU::readAccelPitch() {
    return atan2(-accelData.accelY, accelData.accelZ) * 180.0 / PI;
}

double IMU::readGyroPitch() {
    gyroPitch = gyroPitch - gyroData.gyroX * (dt / 1000.0); // gyro in deg/sec
    return gyroPitch;
}

void IMU::setCalib() {
    imu.calibrateAccelGyro(&calib);
    Serial.println("IMU Calibrated");
    
    Serial.printf(
        "calData IMU::calib = {\n"
        "  .accelBias = {%.3f, %.3f, %.3f},\n"
        "  .gyroBias = {%.3f, %.3f, %.3f}\n"
        "};\n",
        calib.accelBias[0], calib.accelBias[1], calib.accelBias[2],
        calib.gyroBias[0],  calib.gyroBias[1],  calib.gyroBias[2]
    );
}