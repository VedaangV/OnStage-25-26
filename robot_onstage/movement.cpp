//include files
#include "movement.h"
#include "common.h"

using namespace std;

//define motor pins
Motor Motor1{8, 9, A0}, Motor3{10, 11, A1}, Motor2{12, 13, A2}; 

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

float degtorad(int degrees) {
  float radians = degrees * PI / 180;
  return radians;
}

//function: set motor speeds based on velocity
const float radius = 0.346; //ft
const float ftsToSpeed = 157.65;
void vmotor(float Vx, float Vy) {
  int s1 = ftsToSpeed * (-Vx/2 - sqrt(3)*Vy/2 + radius * degtorad(rotation)); //right motor
  int s2 = ftsToSpeed * (-Vx/2 + sqrt(3)*Vy/2 + radius * degtorad(rotation)); //left motor
  int s3 = ftsToSpeed * (Vx + radius * degtorad(rotation)); //back motor

  Serial.print("Left motor: ");
  Serial.println(s1);
  Serial.print("Right motor: ");
  Serial.println(s2);
  Serial.print("Back motor: ");
  Serial.println(s3);
  Serial.print("Rotation: ");
  Serial.println(rotation);
  motor(s1, s2, s3);
}
