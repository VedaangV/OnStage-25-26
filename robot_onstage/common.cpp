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

void fillNPixel(uint32_t color, int count) {
  for(int i = 0; i < count; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 255, 25));
  }
  strip.show();
}

void growNPixel(int ms, uint32_t color, int count){
  int tick = ms / count;
  resetNPixel();
  for(int i = 0; i < count; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 255, 25));
    strip.show();
    delay(tick);
  } 
  fillNPixel(count);
}
void depleteNPixel(int ms, uint32_t color, int count){
  int tick = ms / count;
  fillNPixel(count);
  for(int i = 0; i < count; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, 0));
    strip.show();
    delay(tick);
  } 
  resetNPixel();
}
