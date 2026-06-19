//include guards
#ifndef COMMON_H
#define COMMON_H

//libraries
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

//definitions
#define LED_PIN 22  //Which pin on the board is the NeoPixel attached to?
#define LED_COUNT 30  // How many NeoPixels are attached to the Arduino?
#define BRIGHTNESS 50  // NeoPixel brightness, 0 (min) to 255 (max) // Set BRIGHTNESS to about 1/5 (max = 255)

//variables
extern Adafruit_BNO055 bno;
extern Adafruit_NeoPixel strip;

extern int rotation;
extern int rotation_offset;

inline uint32_t CLEAR = strip.Color(0, 0, 0, 0);
inline uint32_t ICEBLUE = strip.Color(0, 0, 255, 25);
inline uint32_t WHITE = strip.Color(0, 0, 0, 255);
inline uint32_t RED = strip.Color(255, 0, 0, 0);
inline uint32_t GREEN = strip.Color(0, 255, 0, 0);
inline uint32_t BLUE = strip.Color(0, 0, 255, 0);
inline uint32_t YELLOW = strip.Color(255, 255, 0, 0);
inline uint32_t CYAN = strip.Color(0, 255, 255, 0);
inline uint32_t MAGENTA = strip.Color(255, 0, 255, 0);

//functions
void upd_rotation();

void resetNPixel();
void fillNPixel(uint32_t color, int count = LED_COUNT);
void wipeNPixel(int ms, uint32_t color1, uint32_t color2, int count = LED_COUNT);

#endif
