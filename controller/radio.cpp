#include "radio.h"

volatile bool Radio::newTelemetry = false;
TelemetryMessage Radio::lastTelemetry = {};

void Radio::init() {
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSend);

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, botMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("Controller Radio Ready");
}

void Radio::sendPID(const PIDMessage& message) {
    esp_now_send(
        botMAC,
        reinterpret_cast<const uint8_t*>(&message),
        sizeof(PIDMessage)
    );
}

void Radio::sendControl(const ControlMessage& message) {
    esp_now_send(
        botMAC,
        reinterpret_cast<const uint8_t*>(&message),
        sizeof(ControlMessage)
    );
}

bool Radio::hasTelemetry() {
    return newTelemetry;
}

TelemetryMessage Radio::readTelemetry() {
    noInterrupts();
    newTelemetry = false;
    TelemetryMessage copy = lastTelemetry;
    interrupts();
    return copy;
}

void Radio::onReceive(const esp_now_recv_info_t*, const uint8_t* data, int len) {
    if (len < 1) return;

    MessageType type = static_cast<MessageType>(data[0]);

    if (type == MessageType::TELEMETRY && len == sizeof(TelemetryMessage)) {
        memcpy(&lastTelemetry, data, sizeof(TelemetryMessage));
        newTelemetry = true;
    }
}

void Radio::onSend(const wifi_tx_info_t*, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send OK" : "Send FAIL");
}
