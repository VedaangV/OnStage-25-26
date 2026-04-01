#include <WiFi.h>

const char* ssid = "StormingKids";
const char* pswd = "todbot1234";

WiFiClient client;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.printf("Connecting to '%s' with '%s'\n", ssid, pswd);
  WiFi.begin(ssid, pswd);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  digitalWrite(LED_BUILTIN, HIGH);

  client.connect("192.168.32.209", 80);
}

void loop() {
  if (!client.connected()) {
    return;
  }
  client.print("G");
  delay(5000);
}
