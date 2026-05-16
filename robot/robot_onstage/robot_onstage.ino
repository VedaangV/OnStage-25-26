//.h file include
#include "movement.h"

//libraries
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN     22

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT  30

// NeoPixel brightness, 0 (min) to 255 (max)
#define BRIGHTNESS 50 // Set BRIGHTNESS to about 1/5 (max = 255)

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

WiFiServer server(5000); // Listen on port 80
const char* ssid = "StormingKids";
const char* password = "todbot1234";

float vx = 0.0;
float vy = 0.0;
float r = 0.0;

#define LED 21
String inputBuffer = "";
void setup() {
  Serial.begin(115200);
  delay(1000);  
  strip.begin();
  strip.show();

  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();
  pinMode(Motor1.fpin, OUTPUT);
  pinMode(Motor1.rpin, OUTPUT);
  pinMode(Motor1.control, OUTPUT);
  pinMode(Motor2.fpin, OUTPUT);
  pinMode(Motor2.rpin, OUTPUT);
  pinMode(Motor2.control, OUTPUT);
  pinMode(Motor3.fpin, OUTPUT);
  pinMode(Motor3.rpin, OUTPUT);
  pinMode(Motor3.control, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  // Attach interrupt to pin 2, rising edge
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  server.begin();
  digitalWrite(LED_BUILTIN, HIGH);

  resetNPixel();
}

void loop() {

  // Serial.println("Grow");
  // growNPixel(1000);
  // vmotor(4, 4, 0); 
  // delay(1000);
  // Serial.println("Deplete");
  // depleteNPixel(1000);
  // vmotor(-4, -4, 0);
  // delay(1000);

  WiFiClient client = server.available(); // Check for a new client connection
  Serial.println(WiFi.localIP());
  if (client) {
    Serial.println("\nNew Client Connected!");
    while (client.connected()) { // While the client is connected
      if (client.available()) { // If there is data available to read
        char c = client.read(); // Read a character
        if (c == '\n') {
          if(inputBuffer.charAt(0) == 'd'){
            vmotor(0,0,0);
            depleteNPixel(1000);
            inputBuffer = "";
          }
          else if (inputBuffer.charAt(0) == 'c') {
            vmotor(0,0,0);
            growNPixel(1000);
            inputBuffer = "";
          }
          else {
            parseVelocity(inputBuffer);
            Serial.println(vx);
            Serial.println(vy);
            Serial.println(r);
            inputBuffer = "";
            vmotor(vx, vy, r); 
          }
        } 
        else {
          inputBuffer += c;
        } 
      }
    }
    Serial.println("Client Disconnected.");
    client.stop();
  }
}

void parseVelocity(String data) {
  int xpos = data.indexOf("x");
  int ypos = data.indexOf("y");
  int rpos  = data.indexOf("r");
  if (xpos == -1 || ypos == -1 || rpos == -1) return;

  String vxStr = data.substring(data.indexOf("x:") + 3, ypos-3);
  String vyStr = data.substring(data.indexOf("y:") + 3, rpos-2);
  String rStr = data.substring(data.indexOf("r:") + 3, data.length());

  vxStr.trim();
  vyStr.trim();
  rStr.trim();

  vx = vxStr.toFloat();
  vy = vyStr.toFloat();
  r = rStr.toFloat();
}

void resetNPixel() {
  for(int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, 0));
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
}
void depleteNPixel(int ms){
  int tick = ms / LED_COUNT;
  for(int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, 0));
    strip.show();
    delay(tick);
  } 
  resetNPixel();
}
