#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

#include <Arduino.h>
#include "messages.h"

class SerialHandler {
public:
    static void init();
    static void refresh();

    static bool hasControlMessage();
    static bool hasPIDMessage();

    static PIDMessage readPIDMessage();
    static ControlMessage readControlMessage();

    static void writeTelemetryMessage(const TelemetryMessage& message);

private:
    static char* line;

    static bool _hasControlMessage;
    static bool _hasPIDMessage;

    static void handleLine();
};

#endif
