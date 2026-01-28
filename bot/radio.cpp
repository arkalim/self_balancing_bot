#include "radio.h"

volatile bool Radio::newPID = false;
volatile bool Radio::newControl = false;

PIDMessage Radio::lastPID = {};
ControlMessage Radio::lastControl = {};

void Radio::init() {
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSend);

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, controllerMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("Radio Ready");
}

void Radio::sendTelemetry(const TelemetryMessage& message) {
    esp_now_send(
        controllerMAC,
        reinterpret_cast<const uint8_t*>(&message),
        sizeof(TelemetryMessage)
    );
}

bool Radio::hasPID() { return newPID; }

bool Radio::hasControl() { return newControl; }

PIDMessage Radio::getPID() {
    noInterrupts();
    newPID = false;
PIDMessage copy = lastPID;
    interrupts();
    return copy;
}

ControlMessage Radio::getControl() {
    noInterrupts();
    newControl = false;
    ControlMessage copy = lastControl;
    interrupts();
    return copy;
}

void Radio::onReceive(const esp_now_recv_info_t*, const uint8_t* data, int len) {
    if (len < 1) return;

    MessageType type = static_cast<MessageType>(data[0]);

    switch (type) {
        case MessageType::PID:
            if (len == sizeof(PIDMessage)) {
                memcpy(&lastPID, data, sizeof(PIDMessage));
                newPID = true;
            }
            break;

        case MessageType::CONTROL:
            if (len == sizeof(ControlMessage)) {
                memcpy(&lastControl, data, sizeof(ControlMessage));
                newControl = true;
            }
            break;

        default:
            break;
    }
}

void Radio::onSend(const wifi_tx_info_t*, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send OK" : "Send FAIL");
}
