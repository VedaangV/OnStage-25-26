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
  rotation = raw - rotation_offset;
  if (rotation > 360) {
    rotation = rotation - 360;
  }
  if (rotation < 0) {
    rotation = rotation + 360;
  }
  rotation = (rotation > 180) ? (rotation - 360) : rotation; 
}

void resetNPixel() {
  for(int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, 0));
  }
  strip.show();
}

void fillNPixel(uint32_t color, int count) {
  for(int i = 0; i < count; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void wipeNPixel(int ms, uint32_t color1, uint32_t color2, int count){
  int tick = ms / count;
  fillNPixel(color1);
  for(int i = 0; i < count; i++) {
    strip.setPixelColor(i, color2);
    strip.show();
    delay(tick);
  } 
  fillNPixel(color2, count);
}
