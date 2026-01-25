#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <WiFi.h>
#include <esp_now.h>

class Transmitter {
  public:
    struct Message {
      float Kp;
      float Ki;
      float Kd;
    };

    static void init();
    static void send();
    static bool readFromSerial();

  private:
    static Message message;
    static const uint8_t receiverMAC[6];

    static void onSend(const wifi_tx_info_t *info, esp_now_send_status_t status);
};

#endif
