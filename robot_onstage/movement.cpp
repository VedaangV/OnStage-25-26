//include files
#include "movement.h"

//set motor pins
Motor Motor1{8, 9, A0}, Motor2{10, 11, A1}, Motor3{12, 13, A2};

//motor functions
//motor mapping
int cmap(int val, int olow, int ohigh, int mlow, int mhigh) {
  return constrain(map(val, olow, ohigh, mlow, mhigh), mlow, mhigh);
}

//motor speed
void Motor::speed(int val) {
  int map_speed = cmap(abs(val), 0, 100, 0, 255);

  if (val > 0) {
    digitalWrite(fpin, HIGH);
    digitalWrite(rpin, LOW);
  }
  else {
    digitalWrite(fpin, LOW);
    digitalWrite(rpin, HIGH);
  }
  analogWrite(control, map_speed);

}

//function: set motor speeds
void motor(int speed1, int speed2, int speed3) {
  Motor1.speed(speed1);
  Motor2.speed(speed2);
  Motor3.speed(speed3);
}

//function: set motor speeds based on velocity
const int basespeed = 30;
const float radius = 0.058; //meters
const float msToSpeed = 517.24137931;
void vmotor(float Vx, float Vy, float rotation) {
  int s1 = (((-1/2.0)*Vx) + (sqrt(3)/2.0*Vy) + (radius * rotation)); //right motor
  int s3 = (((-1/2.0)*Vx) + (-sqrt(3)/2.0*Vy) + (radius * rotation)); //left motor
  int s2 = (Vx + (radius * rotation)); //back motor

  motor(s1, s2, s3);
}
