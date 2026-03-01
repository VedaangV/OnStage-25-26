//include files
#include "plant_motor.h"

//set motor pins
Motor PMotor{8, 9};

//motor functions
//motor mapping
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

//function: set motor speeds
void motor(int speed) {
  PMotor.speed(speed);
}
