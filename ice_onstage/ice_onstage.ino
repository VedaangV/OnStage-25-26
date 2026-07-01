#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

#define PORT 81

const char* ssid = "jetson";
const char* pswd = "todbot1234";

WiFiServer server(PORT);
WiFiClient client;

#define PIXEL_PIN 16  
#define PIXEL_COUNT 24
Adafruit_NeoPixel ring(PIXEL_COUNT, PIXEL_PIN, NEO_RGBW + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  delay(1000);  
  
  /* Wifi stuff */
  Serial.printf("Connecting to '%s' with '%s'\n", ssid, pswd);
  WiFi.begin(ssid, pswd);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.printf("\nConnected to WiFi\n\nConnect to server at %s:%d\n", WiFi.localIP().toString().c_str(), PORT);
  digitalWrite(LED_BUILTIN, HIGH);

  server.begin();
  Wire.begin();

  ring.begin(); // Initialize NeoPixel ring object (REQUIRED)
  ring.show();  // Initialize all pixels to 'off'
}

const int levels = 6;
int num_active = PIXEL_COUNT;

void deplete() {
  num_active -= PIXEL_COUNT / levels;
  for(int i = 0; i < PIXEL_COUNT; i++) {
    if (i < num_active) {
      ring.setPixelColor(i, ring.Color(0, 0, 125, 175));
    }
    else {
      ring.setPixelColor(i, ring.Color(0, 0, 0, 0));
    }
  } 
  ring.show();
  delay(100);
}

void reset() {
  num_active = PIXEL_COUNT;
  for(int i = 0; i < PIXEL_COUNT; i++) {
    ring.setPixelColor(i, ring.Color(0, 0, 125, 175));
  } 
  ring.show();
}

void loop() {
  Serial.printf("\nConnected to WiFi\n\nConnect to server at %s:%d\n", WiFi.localIP().toString().c_str(), PORT);
  num_active = PIXEL_COUNT;
  for(int i = 0; i < PIXEL_COUNT; i++) {
    ring.setPixelColor(i, ring.Color(0, 0, 125, 175));
  } 
  ring.show();

  //main process
  client = server.accept();
  if (client) {
    while (client.connected()) {
      char req = (char)client.read();
      Serial.printf("Message received: ");
      Serial.printf("%c\n", req);
      if (req == 'D') {
        deplete();
      }
      else if (req == 'R') {
        reset();
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }  
}
