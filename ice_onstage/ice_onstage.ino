#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

#define PORT 81

const char* ssid = "StormingKids";
const char* pswd = "todbot1234";

WiFiServer server(PORT);
WiFiClient client;

#define PIXEL_PIN 16  
#define PIXEL_COUNT 24
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

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

  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'
}

const int levels = 4;
int brightness = 50;
int num_active = PIXEL_COUNT;

void deplete() {
  brightness -= 10;
  num_active -= PIXEL_COUNT / levels;
  for(int i = 0; i < PIXEL_COUNT; i++) {
    if (i < levels) {
      strip.setPixelColor(i, strip.Color(brightness, brightness, brightness));
    }
    else {
      strip.setPixelColor(i, strip.Color(0, 0, 0));
    }
  } 
  strip.show();
}

void loop() {
  for(int i = 0; i < PIXEL_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(brightness, brightness, brightness));
  } 
  strip.show();

  /* Testing w/ wifi*/ 
  client = server.accept();
  if (client) {
    while (client.connected()) {
      char req = (char)client.read();
      Serial.printf("Message received: ");
      Serial.printf("%c\n", req);
      if (req == 'D') {
        deplete();
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }  
}
