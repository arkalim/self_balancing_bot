#include <Arduino.h>
#include "radio.h"

void setup() {
    Serial.begin(115200);
    delay(500);

    Radio::init();
    Serial.println("Enter commands in format: move turn");
    Serial.println("Example: 100 -50");
}

void loop() {
    // Check if serial input is available
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) return;

        int move = 0, turn = 0;

        // Parse two integers from the line
        int spaceIndex = line.indexOf(' ');
        if (spaceIndex > 0) {
            move = line.substring(0, spaceIndex).toInt();
            turn = line.substring(spaceIndex + 1).toInt();

            // Build control message
            ControlMessage cmd{};
            cmd.type = MessageType::CONTROL;
            cmd.move = move;
            cmd.turn = turn;

            // Send over ESP-NOW
            Radio::sendControl(cmd);

            Serial.print("Sent -> Move: ");
            Serial.print(move);
            Serial.print(" | Turn: ");
            Serial.println(turn);
        } else {
            Serial.println("Invalid input. Use: move turn");
        }
    }

    delay(10);
}
