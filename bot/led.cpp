#include "led.h"

Adafruit_NeoPixel LED::led(
    LED::LED_COUNT,
    LED::LED_PIN,
    NEO_GRB + NEO_KHZ800
);

// Color definitions
const uint32_t LED::RED     = LED::led.Color(100, 0,   0);
const uint32_t LED::GREEN   = LED::led.Color(0,   100, 0);
const uint32_t LED::BLUE    = LED::led.Color(0,   0,   100);
const uint32_t LED::YELLOW  = LED::led.Color(100, 100, 0);
const uint32_t LED::CYAN    = LED::led.Color(0,   100, 100);
const uint32_t LED::MAGENTA = LED::led.Color(100, 0,   100);
const uint32_t LED::WHITE   = LED::led.Color(100, 100, 100);
const uint32_t LED::OFF     = LED::led.Color(0,   0,   0);

void LED::init() {
    led.begin();
    led.clear();
    led.show();
}

void LED::glow(uint32_t color) {
    led.setPixelColor(0, color);
    led.show();
}

void LED::pwm(int pwm) {
    pwm = constrain(pwm, 0, 100);
    led.setPixelColor(0, led.Color(pwm, 0,   0));
    led.show();
}
