//.h file include
#include "movement.h"

//libraries
#include <Wire.h>

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
  
}
