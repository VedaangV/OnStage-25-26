//.h file include
#include "movement.h"

//libraries
#include <Wire.h>
float vx = 0.0;
float vy = 0.0;

String inputBuffer = "";
void setup() {
  Serial.begin(115200);
  delay(1000);  

  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  pinMode(20, INPUT);
  while(digitalRead(20));
}

void loop() {
    while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      parseVelocity(inputBuffer);
      inputBuffer = "";
      vmotor(vx, vy, 0.0);  
    } 
    else {
      inputBuffer += c;
    }
  }
  
}

void parseVelocity(String data) {
  int comma = data.indexOf(",");
  if (comma == -1) return;

  String vxStr = data.substring(0, comma);
  String vyStr = data.substring(comma + 1);

  vxStr.trim();
  vyStr.trim();

  vx = vxStr.toFloat();
  vy = vyStr.toFloat();
}
