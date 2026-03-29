#include <WiFi.h>

#define PORT 5000

const char* ssid = "StormingKids";
const char* pswd = "todbot1234";
const char* host = "192.168.4.1";

void setup() {

  Serial.begin(115200);
  delay(1000);

  WiFi.begin(ssid, pswd);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected on: ")
  Serial.println(WiFi.localIP());

}

void loop() {

  WiFiClient client;
  if (client.connect(host, 5000)) {
    client.println("G");
    client.stop();
  }
  delay(5000);

}
