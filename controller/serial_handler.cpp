#include "serial_handler.h"

char* SerialHandler::line;
bool SerialHandler::_hasControlMessage;
bool SerialHandler::_hasPIDMessage;

void SerialHandler::init() {
    Serial.begin(115200);
    Serial.println("Serial handler ready");
}

void SerialHandler::refresh() {
  if (!Serial.available()) return;

  _hasPIDMessage = false;
  _hasControlMessage = false;

  static char buffer[64];
  size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
  buffer[len] = '\0';

  if (len == 0) return;

  line = buffer;
  handleLine();
}

void SerialHandler::handleLine() {
  char cmd[16];
  if (sscanf(line, "%15s", cmd) != 1) return;

  if (strcmp(cmd, "PID") == 0) {
    _hasPIDMessage = true;
  }
  else if (strcmp(cmd, "CONTROL") == 0) {
    _hasControlMessage = true;
  }
  else {
    Serial.println("Unknown command");
  }
}

bool SerialHandler::hasPIDMessage() { return _hasPIDMessage; }

bool SerialHandler::hasControlMessage() { return _hasControlMessage; }

PIDMessage SerialHandler::readPIDMessage() {
  char pidTypeStr[16];
  double kp, ki, kd;

  int parsed = sscanf(
      line,
      "PID %15s %lf %lf %lf",
      pidTypeStr, &kp, &ki, &kd
  );

  PIDMessage message;

  if (parsed != 4) {
      Serial.println("Invalid PID command");
      return message;;
  }

  message.type = MessageType::PID;
  message.kp = kp;
  message.ki = ki;
  message.kd = kd;

  // parse PID type
  message.pidType = PIDType::PITCH; // default case
  if (strcmp(pidTypeStr, "PITCH") == 0)          message.pidType = PIDType::PITCH;
  if (strcmp(pidTypeStr, "VELOCITY") == 0)       message.pidType = PIDType::VELOCITY;
  if (strcmp(pidTypeStr, "VELOCITY_DIFF") == 0)  message.pidType = PIDType::VELOCITY_DIFF;

  return message;
}

ControlMessage SerialHandler::readControlMessage() {
  int move, turn;

  int parsed = sscanf(line, "CONTROL %d %d", &move, &turn);

  ControlMessage message;

  if (parsed != 2) {
      Serial.println("Invalid CONTROL command");
      return message;
  }

  message.type = MessageType::CONTROL;
  message.move = move;
  message.turn = turn;

  return message;
}

void SerialHandler::writeTelemetryMessage(const TelemetryMessage& message) {
  Serial.printf(
    "TELEMETRY "
    "TP=%.3f MP=%.3f "
    "TV=%.3f MV=%.3f "
    "TVD=%.3f MVD=%.3f "
    "PWM=%d PWMD=%d\n",
    message.targetPitch,
    message.measuredPitch,
    message.targetVelocity,
    message.measuredVelocity,
    message.targetVelocityDiff,
    message.measuredVelocityDiff,
    message.pwm,
    message.pwmDiff
  );
}