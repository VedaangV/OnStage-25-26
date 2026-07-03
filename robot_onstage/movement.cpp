//include files
#include "movement.h"
#include "common.h"

using namespace std;

#define ROBOT_0

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

#if defined(ROBOT_5)
  const float mult1 = 0.8;
  const float mult2 = 0.8;
  const float mult3 = 0.8;
#elif defined(ROBOT_0)
  const float mult1 = 1.0;
  const float mult2 = 1.0;
  const float mult3 = 1.0;
#else
  const float mult1 = 1.0;
  const float mult2 = 1.0;
  const float mult3 = 1.0;
#endif

//function: set motor speeds
void motor(int speed1, int speed2, int speed3) {
  Motor1.speed(speed1 * mult1);
  Motor2.speed(speed2 * mult2);
  Motor3.speed(speed3 * mult3);
}

float degtorad(int degrees) {
  float radians = degrees * PI / 180;
  return radians;
}

//function: set motor speeds based on velocity
const float radius = 0.346; //ft
const float ftsToSpeed = 157.65; //ft per sec to motor speed

const float Kp = 6.0;
const float Ki = 0.0;
const float Kd = 4.0;
void vmotor(float Vx, float Vy, float* Vx_act, float* Vy_act) {
  static int perror = 0;
  static int Ierror = 0;

  upd_rotation();

  int rotChange;
  rotChange = Kp*degtorad(rotation) + Kd*degtorad(rotation - perror) + Ki*degtorad(Ierror);  

  int s1 = ftsToSpeed * (-Vx/2 - sqrt(3)*Vy/2 + rotChange); //right motor
  int s2 = ftsToSpeed * (-Vx/2 + sqrt(3)*Vy/2 + rotChange); //left motor
  int s3 = ftsToSpeed * (Vx + rotChange); //back motor
  
  while(1) {
    if (abs(s1) >= 60 || abs(s2) >= 60 || abs(s3) >= 60) {
      s1 = (int)(s1 * 0.9f);
      s2 = (int)(s2 * 0.9f);
      s3 = (int)(s3 * 0.9f);
    }
    else {
      break;
    }
  }

  if (abs(s1) <= 15) {
    s1 = 0;
  }
  if (abs(s2) <= 15) {
    s2 = 0;
  }
  if (abs(s3) <= 15) {
    s3 = 0;
  }

  motor(s1, s2, s3);

  if (Vx_act != nullptr && Vy_act != nullptr && !(Vx_act == 0 && Vy_act == 0)) {
    float m1 = (float)s1 / ftsToSpeed;
    float m2 = (float)s2 / ftsToSpeed;
    float m3 = (float)s3 / ftsToSpeed;

    *Vx_act = (2.0*m3 - m1 - m2) / 3.0;
    *Vy_act = (m2 - m1) / sqrt(3.0);
  }

  perror = rotation;
  if (abs(rotation) < 2) {
    Ierror += rotation;
  }
}
