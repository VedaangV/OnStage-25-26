//file include
#include "common.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

int rotation = 0;
int rotation_offset = 0;
int raw = 0;
void upd_rotation() {
  sensors_event_t event;
  bno.getEvent(&event);
  raw = ((int) event.orientation.x);
  raw = (raw > 180) ? (raw - 360) : raw; 
  rotation = raw - rotation_offset;

}

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
