//include files
#include "movement.h"
#include "common.h"

using namespace std;

//define motor pins
Motor Motor1{8, 9}, Motor2{12, 13}, Motor3{10, 11};

//encoders
const uint8_t direction_pin = 18;
const uint8_t encoder_pin = 19;
volatile int64_t encoders = 0;
void enc_update() {
  if (digitalRead(direction_pin) == HIGH) {
    encoders++;
  }
  else {
    encoders--;
  }
}

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

float degtorad(int degrees) {
  float radians = degrees * PI / 180;
  return radians;
}

//function: set motor speeds based on velocity
const float radius = 0.346; //ft
const float ftsToSpeed = 157.65; //ft per sec to motor speed

const float Kp = 7.0;
const float Ki = 0;
const float Kd = 0;
void vmotor(float Vx, float Vy) {
  static int perror = 0;
  static int Ierror = 0;

  upd_rotation();

  int rotChange;
  if (rotation == 0) {
    rotChange = 0;
    Ierror = 0;
  }
  else {
    rotChange = Kp*degtorad(rotation) + Kd*degtorad(rotation - perror) + Ki*degtorad(Ierror);  
  }

  int s1 = ftsToSpeed * (-Vx/2 - sqrt(3)*Vy/2 + rotChange); //right motor
  int s2 = ftsToSpeed * (-Vx/2 + sqrt(3)*Vy/2 + rotChange); //left motor
  int s3 = ftsToSpeed * (Vx + rotChange); //back motor

  Serial.println(s1);
  Serial.println(s2);
  Serial.println(s3);
  Serial.println(rotation);
  Serial.println(rotChange);

  motor(s1, s2, s3);

  perror = rotation;
  Ierror += rotation;
}
