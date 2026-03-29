#include <Wire.h>
#include <WiFi.h>

#define PORT 5000

const char* ssid = "StormingKids";
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

const int gearDiameter = 5.05; // cm
const float TPR = 960; // 960 ticks per revolution

double circumference = gearDiameter*PI;

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

  /* Wifi stuff */
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("PicoW2");
  Serial.printf("Connecting to '%s' with '%s'\n", ssid, pswd);
  WiFi.begin(ssid, pswd);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.printf("\nConnected to WiFi\n\nConnect to server at %s:%d\n", WiFi.localIP().toString().c_str(), PORT);

  attachInterrupt(digitalPinToInterrupt(ENA), updPos, RISING);

  server.begin();
  Wire.begin();

  client = server.accept();
  while (!client) {
    client = server.accept();
  }

}

float calcDisplacement() {
  return encoderPos * circumference/TPR;
}

void growth() {
  plant.speed(50);
  float initialDisplacement = calcDisplacement();
  float disp = 0;
  int loops = 0;
  while (disp < 1.0) {
    disp = abs(initialDisplacement-calcDisplacement());
    delay(5);
  }
  plant.speed(0);
}

void loop() {
  /* Testing w/ wifi*/
  while (!client.connected()) {
    client = server.accept();
  }
  if (!client.available()) {
    return;
  }
  char req = (char)client.read();
  Serial.printf("Message received: ");
  Serial.printf("%c\n", req);
  if (req == 'G') {
    growth();
  }
  client.stop();
  /* Testing w/out wifi */
  //growth();
  // Serial.printf("Encoder data: %d %d -> %ld\n", digitalRead(ENA), digitalRead(ENB), encoderPos);
  // Serial.printf("Actual position: %f cm\n", encoderPos * circumference/TPR);
  //delay(2000);
}
