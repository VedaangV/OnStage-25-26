//include files
#include "movement.h"

//set motor pins
Motor Motor1{9, 8}, Motor2{11, 10}, Motor3{13, 12};

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
void motor(int speed1, int speed2, int speed3) {
  Motor1.speed(speed1);
  Motor2.speed(speed2);
  Motor3.speed(speed3);
}

//function: set motor speeds based on velocity
const int basespeed = 30;
const int radius = 0.5; //meters

void vmotor(int Vx, int Vy, int rotation) {
  int s1 = (-1/2*Vx) + (sqrt(3)/2*Vy) + (radius * rotation); //top motor
  int s2 = (-1/2*Vx) + (-sqrt(3)/2*Vy) + (radius * rotation); //left motor
  int s3 = Vx + (radius * rotation); //right motor

  motor(s1, s2, s3);
}
