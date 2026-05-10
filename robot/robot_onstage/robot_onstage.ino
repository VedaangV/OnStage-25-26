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
float r = 0.0;

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

  // vmotor(0.4, 0, 0);
  // delay(2000);
  // vmotor(0, 0.4, 0);
  // delay(1500);
  // vmotor(0.4, 0.4, 0);
  // delay(1000);

  // motor(50, 0, 0);
  // delay(1500);
  // motor(0, 50, 0);
  // delay(1000);
  // motor(0, 0, 50);
  // delay(500);

  WiFiClient client = server.available(); // Check for a new client connection
  Serial.println(WiFi.localIP());
  if (client) {
    Serial.println("\nNew Client Connected!");
    while (client.connected()) { // While the client is connected
      if (client.available()) { // If there is data available to read
        char c = client.read(); // Read a character
        if (c == '\n') {
          parseVelocity(inputBuffer);
          Serial.println(vx);
          Serial.println(vy);
          Serial.println(r);
          inputBuffer = "";
          vmotor(vx, vy, r); 
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
