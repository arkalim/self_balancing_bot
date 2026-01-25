#include "transmitter.h"

// RX MAC: 20:6E:F1:6B:97:4C
const uint8_t Transmitter::receiverMAC[6] = {0x20, 0x6E, 0xF1, 0x6B, 0x97, 0x4C};

Transmitter::Message Transmitter::message = {};

void Transmitter::onSend(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    Serial.printf("Sending { %.4f, %.4f, %.4f }\n", message.Kp, message.Ki, message.Kd);
    Serial.print("Send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void Transmitter::init() {
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Transmitter init failed");
    return;
  }

  esp_now_register_send_cb(Transmitter::onSend);

  // Configure receiver (peer)
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = 0;     // auto
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
  }

  Serial.println("Transmitter Ready");
}

void Transmitter::send() {
  esp_now_send(receiverMAC, reinterpret_cast<uint8_t*>(&message), sizeof(message));
}

bool Transmitter::readFromSerial() {
  if (!Serial.available()) { return false; }
  
  String input = Serial.readStringUntil('\n');  // read a line
  input.trim();

  int firstSpace = input.indexOf(' ');
  int secondSpace = input.indexOf(' ', firstSpace + 1);

  if (firstSpace == -1 || secondSpace == -1) return false;

  message.Kp = input.substring(0, firstSpace).todouble();
  message.Ki = input.substring(firstSpace + 1, secondSpace).todouble();
  message.Kd = input.substring(secondSpace + 1).todouble();

  return true;
}