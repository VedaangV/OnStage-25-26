//file include
#include "movement.h"
#include "common.h"

//libraries
#include <Wire.h>
#include <WiFi.h>

//#define ENABLE_WIFI

WiFiServer server(5000); 
const char* ssid = "jetson";
const char* password = "todbot1234";

float vx = 0.0;
float vy = 0.0;

#define LED 21  //LED on Pico W to indicate Wifi connection
String inputBuffer = "";
void setup() {
  Serial.begin(115200);
  delay(1000);  
  strip.begin();
  strip.show();

  Wire.setSDA(16);
  Wire.setSCL(17);
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

  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("IMU not working");
    for (;;);
  }
  Serial.println("IMU working");

#ifdef ENABLE_WIFI
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  server.begin();
  digitalWrite(LED_BUILTIN, HIGH);
#endif

  for(int i = 0; i < 3; i++) {
    fillNPixel();
    delay(750);
    resetNPixel();
    delay(750);
  }
}

void loop() {
#ifdef ENABLE_WIFI
  WiFiClient client = server.available(); // Check for a new client connection
  Serial.println(WiFi.localIP());
  if (client) {
    Serial.println("\nNew Client Connected!");
    while (client.connected()) { // While the client is connected
      if (client.available()) { // If there is data available to read
        char c = client.read(); // Read a character
        if (c == '\n') {
          if(inputBuffer.indexOf("d") != -1){
            inputBuffer = "";
            vmotor(0,0);
            depleteNPixel(3000);
            delay(2000);
          }
          else if (inputBuffer.indexOf("c") != -1) {
            inputBuffer = "";
            vmotor(0,0);
            growNPixel(3000);
            delay(2000);
          }
          else {
            parseVelocity(inputBuffer);
            Serial.println(vx);
            Serial.println(vy);
            Serial.println(r);
            inputBuffer = "";
            vmotor(vx, vy); 
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
#else
  //*** TESTING ***//
  //test individual motor config
  // motor(50, 0, 0);
  // delay(500);
  // motor(0, 50, 0);
  // delay(1000);
  // motor(0, 0, 50);
  // delay(1500);

  //test overall motor config
  vmotor(0.5, 0); //right
  delay(500);
  vmotor(0, 0); 
  delay(2000);
  
  vmotor(-0.5, 0); //left
  delay(1000);
  vmotor(0, 0); 
  delay(2000);
  
  vmotor(0, 0.5); //up
  delay(1500);
  vmotor(0, 0); 
  delay(2000);
  
  vmotor(0, -0.5); //down
  delay(2000);
  vmotor(0, 0); 
  delay(2000);

#endif
}

void parseVelocity(String data) {
  int xpos = data.indexOf("x");
  if (xpos == -1 || ypos == -1) return;

  String vxStr = data.substring(data.indexOf("x:") + 3, ypos-3);
  String vyStr = data.substring(data.indexOf("y:") + 3, data.length());

  vxStr.trim();
  vyStr.trim();

  vx = vxStr.toFloat();
  vy = vyStr.toFloat();
}
