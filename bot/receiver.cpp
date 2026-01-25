#include "receiver.h"

volatile bool Receiver::newMessage = false;
Receiver::Message Receiver::lastMessage = {};

// TX MAC: 20:6E:F1:6B:7A:C8
const uint8_t Receiver::transmitterMAC[6] = {0x20, 0x6E, 0xF1, 0x6B, 0x7A, 0xC8};

void Receiver::init() {
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Receiver init failed");
    return;
  }

  esp_now_register_recv_cb(Receiver::onReceive);
  Serial.println("Receiver Ready");
}

void Receiver::onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Message)) { return; }

  if (memcmp(info->src_addr, transmitterMAC, 6) != 0) { return; }

  memcpy((void*)&lastMessage, data, sizeof(Message));
  newMessage = true;

  Serial.printf("Message received: { %.4f, %.4f, %.4f }\n", lastMessage.Kp, lastMessage.Ki, lastMessage.Kd);
}

bool Receiver::hasNewMessage() {
  return newMessage;
}

Receiver::Message Receiver::getMessage() {
  noInterrupts();
  Message copy = lastMessage;
  newMessage = false;
  interrupts();
  return copy;
}