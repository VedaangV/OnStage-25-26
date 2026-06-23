#include <Servo.h>
#include <Wire.h>
#include <WiFi.h>

#define PORT 80


const char* ssid = "StormingKids";
const char* pswd = "todbot1234";


const int IN1 = 14;  // input 1
const int IN2 = 15;  // input 2

struct Motor {
  uint8_t fpin;
  uint8_t rpin;
  void speed(int val);
};

Motor sPanel{ IN1, IN2 };

int cmap(int val, int olow, int ohigh, int mlow, int mhigh) {
  return constrain(map(val, olow, ohigh, mlow, mhigh), mlow, mhigh);
}




//motor speed
void Motor::speed(int val) {
  int map_speed = cmap(abs(val), 0, 100, 0, 255);

  if (val > 0) {
    analogWrite(fpin, map_speed);
    analogWrite(rpin, 0);
  } else {
    analogWrite(fpin, 0);
    analogWrite(rpin, map_speed);
  }
}

Servo windmill;
const int servoPin = 16;


WiFiServer server(PORT);
WiFiClient client;

void setup() {
  Serial.begin(115200);
  delay(1000);

  windmill.attach(servoPin);

  /* Wifi stuff */
  Serial.printf("Connecting to '%s' with '%s'\n", ssid, pswd);
  WiFi.begin(ssid, pswd);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.printf("\nConnected to WiFi\n\nConnect to server at %s:%d\n", WiFi.localIP().toString().c_str(), PORT);



  server.begin();
  Wire.begin();
}



void shake() {
  sPanel.speed(30);
  windmill.attach(servoPin);
  windmill.write(0);
}

void stopShake(){
  sPanel.speed(0);
  windmill.detach();
}

void loop() {
  /* Testing w/ wifi*/
  client = server.accept();
  if (client) {
    while (client.connected()) {
      char req = (char)client.read();
      Serial.printf("Message received: ");
      Serial.printf("%c\n", req);
      if (req == 'S') {
        shake();
      }else if (req == 'T'){
        stopShake();
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
  /* Testing w/out wifi */
  // shake();
  // delay(1000);
  // stopShake();`
}