#include <Wire.h>
#include <WiFi.h>

#define PORT 80

const char* ssid = "jetson";
const char* pswd = "todbot1234";

const int ENA = 14; // encoder 1
const int ENB = 15; // encoder 2
const int IN1 = 16; // input 1
const int IN2 = 17; // input 2

struct Motor {
  uint8_t fpin;
  uint8_t rpin;
  void speed(int val);
};

Motor plant{IN2, IN1};

int cmap(int val, int olow, int ohigh, int mlow, int mhigh) {
  return constrain(map(val, olow, ohigh, mlow, mhigh), mlow, mhigh);
}

float encoderPos = 0;
const float stage_encoders = 160;
float current_encoders = 0;

//motor speed
void Motor::speed(int val) {
  int map_speed = cmap(abs(val), 0, 100, 0, 255);

  if (val > 0) {
    analogWrite(fpin, map_speed);
    analogWrite(rpin, 0);
  }
  else {
    analogWrite(fpin, 0);
    analogWrite(rpin, map_speed);
  }
}

void updPos() {
  if (digitalRead(ENB) == HIGH) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

WiFiServer server(PORT);
WiFiClient client;

void setup() {
  Serial.begin(115200);
  delay(1000);  

  pinMode(ENA, INPUT_PULLUP);
  pinMode(ENB, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  /* Wifi stuff */
  Serial.printf("Connecting to '%s' with '%s'\n", ssid, pswd);
  WiFi.begin(ssid, pswd);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.printf("\nConnected to WiFi\n\nConnect to server at %s:%d\n", WiFi.localIP().toString().c_str(), PORT);
  digitalWrite(LED_BUILTIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENA), updPos, RISING);

  server.begin();
  Wire.begin();

}

void growth() {
  plant.speed(30);
  while (encoderPos < (current_encoders + stage_encoders)) {
    delay(2);
  }
  plant.speed(0);
  current_encoders = encoderPos;
}

void loop() {
  //main
  Serial.printf("\nConnected to WiFi\n\nConnect to server at %s:%d\n", WiFi.localIP().toString().c_str(), PORT);
  client = server.accept();
  if (client) {
    while (client.connected()) {
      char req = (char)client.read();
      Serial.printf("Message received: ");
      Serial.printf("%c\n", req);
      if (req == 'G') {
        growth();
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }

}
