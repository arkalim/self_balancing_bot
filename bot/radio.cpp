#include "radio.h"

volatile bool Radio::newMessage = false;
uint8_t Radio::lastMessage[250] = {}; // adjust size for your largest struct

void Radio::init() {
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }

    // RX + TX callbacks
    esp_now_register_recv_cb(Radio::onReceive);
    esp_now_register_send_cb(Radio::onSend);

    // Add default peer
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, peerMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("Radio Ready");
}

void Radio::onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    memcpy(lastMessage, data, len);
    newMessage = true;

    Serial.printf("Received %d bytes\n", len);
}

void Radio::onSend(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    Serial.print("Send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

template <typename T>
bool Radio::readFromSerial(T &msg) {
    if (!Serial.available()) return false;

    String input = Serial.readStringUntil('\n');
    input.trim();

    // Simple parser for space-separated values
    int start = 0;
    const char *p = input.c_str();
    size_t fieldIndex = 0;

    for (size_t i = 0; i < sizeof(T) / sizeof(double); i++) {
        char *endptr;
        double val = strtod(p + start, &endptr);
        if (endptr == p + start) return false; // parsing failed

        ((double*)&msg)[i] = val;
        start = endptr - p + 1;
    }

    return true;
}
