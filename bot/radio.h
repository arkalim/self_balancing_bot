#ifndef RADIO_H
#define RADIO_H

#include <WiFi.h>
#include <esp_now.h>
#include "messages.h"

class Radio {
public:
    static void init();

    // Sending
    static void sendTelemetry(const TelemetryMessage& message);

    // Receiving
    static bool hasPID();
    static bool hasControl();

    static PIDMessage getPID();
    static ControlMessage getControl();

    static constexpr uint8_t controllerMAC[6] = { 0x20, 0x6E, 0xF1, 0x6E, 0x08, 0x80 };

private:
    static void onReceive(const esp_now_recv_info_t*, const uint8_t*, int);
    static void onSend(const wifi_tx_info_t*, esp_now_send_status_t);

    static volatile bool newPID;
    static volatile bool newControl;

    static PIDMessage lastPID;
    static ControlMessage lastControl;
};

#endif
