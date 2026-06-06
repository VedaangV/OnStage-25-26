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

//functions
void upd_rotation();

void resetNPixel();
void fillNPixel();
void growNPixel(int);
void depleteNPixel(int);
