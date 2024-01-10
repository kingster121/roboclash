#include <Arduino.h>
#include <FastLED.h>

#define LED_PIN     14
#define LED_PIN_RED 26
#define LED_PIN_BLUE 27

#define NUM_LEDS    17

CRGB leds[NUM_LEDS];
CRGB leds_red[NUM_LEDS];
CRGB leds_blue[NUM_LEDS];

void setup() {
    FastLED.addLeds<WS2812, LED_PIN, RGB>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN_RED, RGB>(leds_red, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN_BLUE, RGB>(leds_blue, NUM_LEDS);    

}

void loop() {
    for (int i = 0; i <= 17; i++) {
        int colour1 = rand() % 256;
        int colour2 = rand() % 256;
        int colour3 = rand() % 256;
        leds[i] = CRGB (colour1, colour2, colour3);
        leds_red[i] = CRGB (0, 255, 0);
        leds_blue[i] = CRGB (0, 0, 255);
    }
    FastLED.show();
    delay(1000);
}