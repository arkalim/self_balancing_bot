#ifndef RADIO_H
#define RADIO_H

#include <WiFi.h>
#include <esp_now.h>

class Radio {
public:
    static void init();

    template <typename T>
    static void send(const uint8_t *mac, const T &data) {
        esp_now_send(mac,
            reinterpret_cast<const uint8_t*>(&data),
            sizeof(T)
        );
    }

    static bool hasNewMessage() { return newMessage; }

    template <typename T>
    static T getMessage() {
        noInterrupts();
        T copy;
        memcpy(&copy, lastMessage, sizeof(T));
        newMessage = false;
        interrupts();
        return copy;
    }

    template <typename T>
    static bool readFromSerial(T &msg) {
        if (!Serial.available()) return false;

        String input = Serial.readStringUntil('\n');
        input.trim();

        int i1 = input.indexOf(' ');
        int i2 = input.indexOf(' ', i1 + 1);
        if (i1 == -1 || i2 == -1) return false;

        msg.Kp = input.substring(0, i1).toDouble();
        msg.Ki = input.substring(i1 + 1, i2).toDouble();
        msg.Kd = input.substring(i2 + 1).toDouble();

        return true;
    }

    static constexpr uint8_t peerMAC[6] = {
        0x20, 0x6E, 0xF1, 0x6B, 0x97, 0x4C
    };

private:
    static void onReceive(const esp_now_recv_info_t*, const uint8_t*, int);
    static void onSend(const wifi_tx_info_t*, esp_now_send_status_t);

    static volatile bool newMessage;
    static uint8_t lastMessage[250];
};

#endif
