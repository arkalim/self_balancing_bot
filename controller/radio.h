#ifndef RADIO_H
#define RADIO_H

#include <WiFi.h>
#include <esp_now.h>
#include "messages.h"

class Radio {
public:
    static void init();

    // Sending
    static void sendPID(const PIDMessage& message);
    static void sendControl(const ControlMessage& message);

    // Receiving
    static bool hasTelemetry();
    static TelemetryMessage readTelemetry();

    static constexpr uint8_t botMAC[6] = {0x20, 0x6E, 0xF1, 0x6B, 0x97, 0x4C};

private:
    static void onReceive(const esp_now_recv_info_t*, const uint8_t*, int);
    static void onSend(const wifi_tx_info_t*, esp_now_send_status_t);

    static volatile bool newTelemetry;
    static TelemetryMessage lastTelemetry;
};

#endif
