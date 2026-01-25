#ifndef RADIO_H
#define RADIO_H

#include <WiFi.h>
#include <esp_now.h>

class Radio {
public:
    static void init();

    // Send a message of any struct type
    template <typename T>
    static void send(const uint8_t *mac, const T &data) {
        esp_err_t result = esp_now_send(mac, reinterpret_cast<const uint8_t*>(&data), sizeof(T));
        Serial.printf("Sent %u bytes\n", sizeof(T));
        if (result != ESP_OK) {
            Serial.println("Send failed");
        }
    }

    static bool hasNewMessage() { return newMessage; }

    // Get the last received message as a struct of type T
    template <typename T>
    static T getMessage() {
        noInterrupts();
        T copy;
        memcpy(&copy, &lastMessage, sizeof(T));
        newMessage = false;
        interrupts();
        return copy;
    }

    // Read message from Serial into struct T
    template <typename T>
    static bool readFromSerial(T &msg);

    static constexpr uint8_t peerMAC[6] = {0x20, 0x6E, 0xF1, 0x6B, 0x7A, 0xC8};

private:
    static volatile bool newMessage;
    static uint8_t lastMessage[250]; // buffer large enough for any struct

    // ESP-NOW callbacks
    static void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len);
    static void onSend(const wifi_tx_info_t *info, esp_now_send_status_t status);
};

#endif
