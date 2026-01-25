#include "motors.h"

unsigned int Motors::sampleTime = 0;
unsigned long Motors::lastTime = 0;
unsigned long Motors::dt = 0;
double Motors::velocity = 0;

bool Motors::power;
int Motors::leftDir = 1;
int Motors::rightDir = 1;
volatile long Motors::leftEncoderCount = 0;
volatile long Motors::rightEncoderCount = 0;
long Motors::lastLeftEncoderCount = 0;
long Motors::lastRightEncoderCount = 0;

void IRAM_ATTR leftEncoderISR() { Motors::leftEncoderCount += Motors::leftDir; }
void IRAM_ATTR rightEncoderISR() { Motors::rightEncoderCount += Motors::rightDir; }

void Motors::init(unsigned int sampleTime, bool power) {
    Motors::sampleTime = sampleTime;
    lastTime = millis();

    Motors::power = power;

    pinMode(LEFT_DIR1_PIN, OUTPUT);
    pinMode(LEFT_DIR2_PIN, OUTPUT);
    pinMode(RIGHT_DIR1_PIN, OUTPUT);
    pinMode(RIGHT_DIR2_PIN, OUTPUT);

    ledcAttach(LEFT_PWM_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(RIGHT_PWM_PIN, PWM_FREQ, PWM_RES);

    pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightEncoderISR, RISING);
}

// ================= HIGH-LEVEL MOVEMENTS =================
void Motors::move(int pwm) {
    setLeftMotor(pwm);
    setRightMotor(pwm);
}

void Motors::turn(int pwm) {
    setLeftMotor(pwm);
    setRightMotor(-pwm);
}

void Motors::stop() {
    setLeftMotor(0);
    setRightMotor(0);
}

// ================= LOW-LEVEL MOTOR CONTROL =================
void Motors::setLeftMotor(int pwm) {
    if (!power) {
        digitalWrite(LEFT_DIR1_PIN, LOW);
        digitalWrite(LEFT_DIR2_PIN, LOW);
        return;
    }

    pwm = constrain(pwm, -255, 255);

    if (pwm >= 0) {
        leftDir = 1;
        digitalWrite(LEFT_DIR1_PIN, HIGH);
        digitalWrite(LEFT_DIR2_PIN, LOW);
    } else {
        leftDir = -1;
        digitalWrite(LEFT_DIR1_PIN, LOW);
        digitalWrite(LEFT_DIR2_PIN, HIGH);
        pwm = -pwm;
    }

    ledcWrite(LEFT_PWM_PIN, pwm);
}

void Motors::setRightMotor(int pwm) {
    if (!power) {
        digitalWrite(RIGHT_DIR1_PIN, LOW);
        digitalWrite(RIGHT_DIR2_PIN, LOW);
        return;
    }

    pwm = constrain(pwm, -255, 255);

    if (pwm >= 0) {
        rightDir = 1;
        digitalWrite(RIGHT_DIR1_PIN, HIGH);
        digitalWrite(RIGHT_DIR2_PIN, LOW);
    } else {
        rightDir = -1;
        digitalWrite(RIGHT_DIR1_PIN, LOW);
        digitalWrite(RIGHT_DIR2_PIN, HIGH);
        pwm = -pwm;
    }

    ledcWrite(RIGHT_PWM_PIN, pwm);
}

// ================= ENCODER HELPERS =================
bool Motors::ready() {
    unsigned long now = millis();
    dt = now - lastTime;
    if (dt < sampleTime) { return false; }
    lastTime = now;

    // read atomically
    long leftEnc, rightEnc;
    noInterrupts();
    leftEnc  = leftEncoderCount;
    rightEnc = rightEncoderCount;
    interrupts();

    long leftEncPulses = leftEnc - lastLeftEncoderCount;
    long rightEncPulses = rightEnc - lastRightEncoderCount;

    lastLeftEncoderCount  = leftEncoderCount;
    lastRightEncoderCount = rightEncoderCount;

    double encoderPulses = (leftEncPulses + rightEncPulses) * 0.5;
    double revs = encoderPulses / ENC_PULSE_PER_REV;
    velocity = revs / (dt / 1000.0); // rps
    return true;
}

double Motors::readVelocity() {
    return velocity;
}