#include "motors.h"

unsigned int Motors::sampleTime = 0;
unsigned long Motors::lastTime = 0;
unsigned long Motors::dt = 0;

bool Motors::power;
volatile long Motors::leftEncoderCount = 0;
volatile long Motors::rightEncoderCount = 0;
long Motors::lastLeftEncoderCount = 0;
long Motors::lastRightEncoderCount = 0;
double Motors::leftVelocity = 0;
double Motors::rightVelocity = 0;

void IRAM_ATTR leftEncoderISR() {
    Motors::leftEncoderCount += (digitalRead(Motors::LEFT_ENC2_PIN) ? 1 : -1);
}
void IRAM_ATTR rightEncoderISR() {
    Motors::rightEncoderCount += (digitalRead(Motors::RIGHT_ENC2_PIN) ? 1 : -1);
}

void Motors::init(unsigned int sampleTime, bool power) {
    Motors::sampleTime = sampleTime;
    lastTime = millis();

    Motors::power = power;

    ledcAttach(LEFT_DIR1_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(LEFT_DIR2_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(RIGHT_DIR1_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(RIGHT_DIR2_PIN, PWM_FREQ, PWM_RES);

    pinMode(LEFT_ENC1_PIN, INPUT_PULLUP);
    pinMode(LEFT_ENC2_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENC1_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENC2_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC1_PIN), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC1_PIN), rightEncoderISR, RISING);
}

int Motors::applyDeadband(int pwm) {
    if (pwm == 0) return 0; // don't move if zero

    pwm = constrain(pwm, -255, 255);

    int sign = (pwm >= 0) ? 1 : -1;
    int absPWM = abs(pwm);

    // map full 0..255 → MOTOR_DEADBAND..255
    absPWM = map(absPWM, 0, 255, DEADBAND, 255);

    return sign * absPWM;
}

void Motors::setLeftMotor(int pwm) {
    if (!power || pwm == 0) {
        ledcWrite(LEFT_DIR1_PIN, 0);
        ledcWrite(LEFT_DIR2_PIN, 0);
        return;
    }

    pwm = applyDeadband(pwm);

    if (pwm > 0) {
        ledcWrite(LEFT_DIR1_PIN, pwm);
        ledcWrite(LEFT_DIR2_PIN, 0);
    } else {
        ledcWrite(LEFT_DIR1_PIN, 0);
        ledcWrite(LEFT_DIR2_PIN, -pwm);
    }
}

void Motors::setRightMotor(int pwm) {
    if (!power || pwm == 0) {
        ledcWrite(RIGHT_DIR1_PIN, 0);
        ledcWrite(RIGHT_DIR2_PIN, 0);
        return;
    }

    pwm = applyDeadband(pwm);

    if (pwm > 0) {
        ledcWrite(RIGHT_DIR1_PIN, pwm);
        ledcWrite(RIGHT_DIR2_PIN, 0);
    } else {
        ledcWrite(RIGHT_DIR1_PIN, 0);
        ledcWrite(RIGHT_DIR2_PIN, -pwm);
    }
}

bool Motors::newVelocity() {
    unsigned long now = millis();
    dt = now - lastTime;
    if (dt < sampleTime) return false;
    lastTime = now;

    // atomic read
    long leftEnc, rightEnc;
    noInterrupts();
    leftEnc  = leftEncoderCount;
    rightEnc = rightEncoderCount;
    interrupts();

    long leftPulses  = leftEnc  - lastLeftEncoderCount;
    long rightPulses = rightEnc - lastRightEncoderCount;

    lastLeftEncoderCount  = leftEnc;
    lastRightEncoderCount = rightEnc;

    double dt_s = dt / 1000.0;

    // pulses → revolutions → rps
    leftVelocity  = (leftPulses  / (double)ENC_PULSE_PER_REV) / dt_s;
    rightVelocity = (rightPulses / (double)ENC_PULSE_PER_REV) / dt_s;

    return true;
}

double Motors::readVelocity() {
    return 0.5 * (leftVelocity + rightVelocity);
}

double Motors::readVelocityDiff() {
    return leftVelocity - rightVelocity;
}

void Motors::move(int pwm, int pwmDiff) {
    setLeftMotor(pwm + pwmDiff);
    setRightMotor(pwm - pwmDiff);
}

