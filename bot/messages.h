#pragma once

enum class MessageType : uint8_t {
    TELEMETRY  = 1,
    PID        = 2,
    CONTROL    = 3
};

struct TelemetryMessage {
    MessageType type = MessageType::TELEMETRY;
    double targetPitch;
    double measuredPitch;
    double targetVelocity;
    double measuredVelocity;
    int pwm;
};

struct PIDMessage {
    MessageType type = MessageType::PID;
    double kp;
    double ki;
    double kd;
};

struct ControlMessage {
    MessageType type = MessageType::CONTROL;
    int move;
    int turn;
};