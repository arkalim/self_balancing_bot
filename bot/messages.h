#pragma once

enum class MessageType : uint8_t {
    TELEMETRY  = 1,
    PID        = 2,
    CONTROL    = 3
};

enum class PIDType : uint8_t {
    PITCH           = 1,
    VELOCITY        = 2,
    VELOCITY_DIFF   = 3
};

struct TelemetryMessage {
    MessageType type = MessageType::TELEMETRY;
    double targetPitch;
    double measuredPitch;
    double targetVelocity;
    double measuredVelocity;
    double targetVelocityDiff;
    double measuredVelocityDiff;
    int pwm;
    int pwmDiff;
};

struct PIDMessage {
    MessageType type = MessageType::PID;
    PIDType pidType;
    double kp;
    double ki;
    double kd;
};

struct ControlMessage {
    MessageType type = MessageType::CONTROL;
    int move;
    int turn;
};