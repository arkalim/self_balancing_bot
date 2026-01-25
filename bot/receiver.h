#ifndef RECEIVER_H
#define RECEIVER_H

#include <WiFi.h>
#include <esp_now.h>

class Receiver {
  public:
    struct Message {
      float Kp;
      float Ki;
      float Kd;
    };

    static const uint8_t transmitterMAC[6];

    static void init();
    static bool hasNewMessage();
    static Message getMessage();

  private:
    static Message lastMessage;
    static volatile bool newMessage;

    static void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len);
};

#endif
