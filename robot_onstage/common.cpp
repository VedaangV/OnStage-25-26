//file include
#include "common.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

void resetNPixel() {
  for(int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, 0));
  }
  strip.show();
}

void fillNPixel() {
  for(int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 255, 25));
  }
  strip.show();
}

void growNPixel(int ms){
  int tick = ms / LED_COUNT;
  resetNPixel();
  for(int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 255, 25));
    strip.show();
    delay(tick);
  } 
  fillNPixel();
}
void depleteNPixel(int ms){
  int tick = ms / LED_COUNT;
  fillNPixel();
  for(int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, 0));
    strip.show();
    delay(tick);
  } 
  resetNPixel();
}
