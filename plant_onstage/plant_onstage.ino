#include <Wire.h>
#include <Arduino.h>

#define IN1 X; // input 1
#define IN2 Y; // input 2

String inputBuffer = "";
void setup() {
  Serial.begin(115200);
  delay(1000);  

  Wire.begin();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  
}

void growth() {
  digitalWrite(IN1, HIGH);
  delay(1000);
  digitalWrite(IN1, LOW);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'G') {
      growth();
    }
  }
}


