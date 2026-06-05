//include files
#include "movement.h"
#include "common.h"

using namespace std;

//*** FIX ***//
#define ROBOT_152

//set motor pins
#ifdef ROBOT_152
  //***FOLLOW THIS CONFIGURATION, THIS IS THE ROBOT WITH THE PICO 2W AND THE GREEN COVER ON THE TOP***//
  Motor Motor1{8, 9, A0}, Motor3{10, 11, A1}, Motor2{12, 13, A2}; //pico 2w .152

#elif defined(ROBOT_146)
  Motor Motor2{8, 9, A0}, Motor3{10, 11, A1}, Motor1{12, 13, A2}; //pico w .146
#else
#endif
//******//

#define CORRECT_ROT  //if defined, robot rotates to maintain 0 deg, else adjust formula and do not correct rotation

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
const float Kp = 1.0;
const float Kd = 0.0;
static float prev_rot;
void vmotor(float Vx, float Vy) {
  sensors_event_t event;
  bno.getEvent(&event);
  int rotation;
  rotation = ((int) event.orientation.x);

#ifdef CORRECT_ROT
  int s1 = ftsToSpeed * (-Vx/2 - sqrt(3)*Vy/2 + radius * degtorad(rotation)*Kp + radius*degtorad(rotation - prev_rot)*Kd); //right motor
  int s2 = ftsToSpeed * (-Vx/2 + sqrt(3)*Vy/2 + radius * degtorad(rotation)*Kp + radius*degtorad(rotation - prev_rot)*Kd); //left motor
  int s3 = ftsToSpeed * (Vx + radius * degtorad(rotation)*Kp + radius*degtorad(rotation - prev_rot)*Kd); //back motor
#else
  int s1 = ftsToSpeed * (Vx*cos(degtorad(rotation+60)) + Vy*sin(degtorad(rotation+60)); //right motor
  int s2 = ftsToSpeed * (Vx*cos(degtorad(rotation+300)) + Vy*sin(degtorad(rotation+300)); //left motor
  int s3 = ftsToSpeed * (Vx*cos(degtorad(rotation+180)) + Vy*sin(degtorad(rotation+180)); //back motor
#endif

  Serial.print("Left motor: ");
  Serial.println(s1);
  Serial.print("Right motor: ");
  Serial.println(s2);
  Serial.print("Back motor: ");
  Serial.println(s3);
  motor(s1, s2, s3);

  prev_rot = rotation;
}
