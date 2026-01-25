#include "radio.h"

volatile bool Radio::newMessage = false;
uint8_t Radio::lastMessage[250] = {};

void Radio::init() {
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSend);

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, peerMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("Radio Ready");
}

void Radio::onReceive(const esp_now_recv_info_t*, const uint8_t *data, int len) {
    memcpy(lastMessage, data, len);
    newMessage = true;
}

void Radio::onSend(const wifi_tx_info_t*, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send OK" : "Send FAIL");
}
