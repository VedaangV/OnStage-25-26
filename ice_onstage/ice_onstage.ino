#include <Wire.h>
#include <WiFi.h>

#define PORT 

const char* ssid = "StormingKids";
const char* pswd = "todbot1234";

WiFiServer server(PORT);
WiFiClient client;

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
}

void deplete() {
  //deplete ice code here
}

void loop() {
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
