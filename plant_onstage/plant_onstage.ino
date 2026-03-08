#include <Wire.h>

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

String inputBuffer = "";
void setup() {
  Serial.begin(9600);
  delay(1000);  

  Wire.begin();

}

void growth() {
  plant.speed(25);
  delay(1000);
  plant.speed(0);
  delay(1000);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'G') {
      growth();
    }
  }
}
