//.h file include
#include "movement.h"

//libraries
#include <Wire.h>
#include <WiFi.h>
WiFiServer server(5000); // Listen on port 80
const char* ssid = "StormingKids";
const char* password = "todbot1234";


float vx = 0.0;
float vy = 0.0;

#define LED 21
String inputBuffer = "";
void setup() {
  Serial.begin(115200);
  delay(1000);  

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
  delay(500);




}

void loop() {

  WiFiClient client = server.available(); // Check for a new client connection
  Serial.println(WiFi.localIP());
  if (client) {
    Serial.println("\nNew Client Connected!");
    while (client.connected()) { // While the client is connected
      if (client.available()) { // If there is data available to read
        char c = client.read(); // Read a character
        if (c == '\n') {
          parseVelocity(inputBuffer);
          Serial.print(vx);
          Serial.println(vy);
          inputBuffer = "";
          vmotor(vx, vy, 0); 
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
  int comma = data.indexOf(",");
  if (comma == -1) return;

  String vxStr = data.substring(data.indexOf("x:") + 3, comma);
  String vyStr = data.substring(data.indexOf("y:") + 3, data.length());

  vxStr.trim();
  vyStr.trim();

  vx = vxStr.toFloat();
  vy = vyStr.toFloat();
}
