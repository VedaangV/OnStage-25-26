#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

#define PORT 80

const char* ssid = "iptime";
const char* pswd = "srdevhelp";

WiFiServer server(PORT);
WiFiClient client;

#define LED_PIN 16  //Which pin on the board is the NeoPixel attached to?
#define LED_COUNT 30  // How many NeoPixels are attached to the Arduino?
#define BRIGHTNESS 50  // NeoPixel brightness, 0 (min) to 255 (max) // Set BRIGHTNESS to about 1/5 (max = 255)
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

int currentLED = 29;
int currentBlue = 255; 

void resetNPixel() {
  for(int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, 0));
  }
  strip.show();
}

void fillNPixel(int blue, int COUNT) {
  for(int i = 0; i < COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, blue, 25));
  }
  strip.show();
}
void depleteNPixel(int LED_NUM){
  int LED_TARGET = currentLED-LED_NUM;
  //int tick = ms / hi;
  
  for(int i = currentLED; i >= LED_TARGET; i-- ){
    strip.setPixelColor(i, strip.Color(0, 0, 0, 0));
    strip.show();
    delay(50);
  } 
  currentLED = LED_TARGET;
  currentBlue -= 80;
  fillNPixel(currentBlue,LED_TARGET);
  
  //resetNPixel();
}
void growNPixel(int LED_NUM){
  int LED_TARGET = currentLED+LED_NUM;

   currentBlue+=80;
    fillNPixel(currentBlue,currentLED);
  for(int i = currentLED; i <= LED_TARGET; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, currentBlue, 25));
    strip.show();
    delay(50);
  } 
   
    currentLED = LED_TARGET;

  
 // fillNPixel();
}
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

  strip.begin(); // Initialize NeoPixel ring object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'
  fillNPixel(255,LED_COUNT);
}



void loop() {

  
 

  //*** testing ***//
  // delay(1000);
  // for(int i = 0; i < levels; i++) {
  //   deplete();
  //   delay(1000);
  // }
  // for(int i = 0; i < PIXEL_COUNT; i++) {
  //   ring.setPixelColor(i, ring.Color(0, 0, 0, 0));
  // } 
  // while(1);
  //*** testing ***//

  //main process
  client = server.accept();
  if (client) {
    while (client.connected()) {
      char req = (char)client.read();
      Serial.printf("Message received: ");
      Serial.printf("%c\n", req);
      if (req == 'C') {
        growNPixel(10);
      }else if (req == 'D'){
          depleteNPixel(10);
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }  
  

}