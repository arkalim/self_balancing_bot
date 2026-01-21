#include "motors.h"

void Motors::init() {
    pinMode(LEFT_DIR1_PIN, OUTPUT);
    pinMode(LEFT_DIR2_PIN, OUTPUT);
    pinMode(RIGHT_DIR1_PIN, OUTPUT);
    pinMode(RIGHT_DIR2_PIN, OUTPUT);

    ledcAttach(LEFT_PWM_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(RIGHT_PWM_PIN, PWM_FREQ, PWM_RES);
}

// ================= LOW-LEVEL MOTOR CONTROL =================
void Motors::setLeftMotor(int speed) {
    speed = constrain(speed, -255, 255);

    if (speed >= 0) {
        digitalWrite(LEFT_DIR1_PIN, HIGH);
        digitalWrite(LEFT_DIR2_PIN, LOW);
    } else {
        digitalWrite(LEFT_DIR1_PIN, LOW);
        digitalWrite(LEFT_DIR2_PIN, HIGH);
        speed = -speed;
    }

    ledcWrite(LEFT_PWM_PIN, speed);
}

void Motors::setRightMotor(int speed) {
    speed = constrain(speed, -255, 255);

    if (speed >= 0) {
        digitalWrite(RIGHT_DIR1_PIN, HIGH);
        digitalWrite(RIGHT_DIR2_PIN, LOW);
    } else {
        digitalWrite(RIGHT_DIR1_PIN, LOW);
        digitalWrite(RIGHT_DIR2_PIN, HIGH);
        speed = -speed;
    }

    ledcWrite(RIGHT_PWM_PIN, speed);
}

// ================= HIGH-LEVEL MOVEMENTS =================
void Motors::moveForward(int speed) {
    setLeftMotor(speed);
    setRightMotor(speed);
}

void Motors::moveBackward(int speed) {
    setLeftMotor(-speed);
    setRightMotor(-speed);
}

void Motors::turnLeft(int speed) {
    setLeftMotor(-speed);
    setRightMotor(speed);
}

void Motors::turnRight(int speed) {
    setLeftMotor(speed);
    setRightMotor(-speed);
}

void Motors::stop() {
    setLeftMotor(0);
    setRightMotor(0);
}
