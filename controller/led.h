#ifndef LED_H
#define LED_H

#include <Adafruit_NeoPixel.h>

class LED {
  public:
    static const int LED_PIN = 10;
    static const int LED_COUNT = 1;
    static Adafruit_NeoPixel led;

    // Named colors (24-bit RGB)
    static const uint32_t RED;
    static const uint32_t GREEN;
    static const uint32_t BLUE;
    static const uint32_t YELLOW;
    static const uint32_t CYAN;
    static const uint32_t MAGENTA;
    static const uint32_t WHITE;
    static const uint32_t OFF;

    static void init();
    static void glow(uint32_t color);
    static void pwm(int pwm);
};

#endif
