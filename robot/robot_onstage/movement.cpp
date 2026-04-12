//include files
#include "movement.h"

//set motor pins
Motor Motor2{8, 9, A0}, Motor3{10, 11, A1}, Motor1{12, 13, A2};

const int encoderPinA = 3;
const int encoderPinB = 2;
volatile long encoderTicks = 0;


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
  float fmults[] = {1.4, 1.2, 1};
  float bmults[] = {1.5, 1.5, 1};
  
  Motor1.speed(speed1*((speed1>0) ? fmults[0] : bmults[0]));
  Motor2.speed(speed2*((speed2>0) ? fmults[1] : bmults[1]));
  Motor3.speed(speed3*((speed3>0) ? fmults[2] : bmults[2]));
}

//function: set motor speeds based on velocity
const int basespeed = 30;
const float radius = 0.058; //meters
const float msToSpeed = 517.24137931;
void vmotor(float Vx, float Vy, float rotation) {
  
  int s1 = (((-1/2.0)*Vx) - (sqrt(3.0)/2.0*Vy) + (radius * rotation)); //right motor
  int s2 = (((-1/2.0)*Vx) + (sqrt(3.0)/2.0*Vy) + (radius * rotation)); //left motor
  int s3 = (Vx + (radius * rotation)); //back motor

  motor(s1, s2, s3);
}

void handleEncoder() {
  if (digitalRead(encoderPinB) == HIGH) {
    encoderTicks++; // CW
  } else {
    encoderTicks--; // CCW
  }
}

/*
void moveRPM(Motor motor, float rpm, float kp, float ki, float kd){
  if(rpm != motor.rpm){
    motor.rpm = rpm;
    motor.current_speed = rpm*2-20;
    motor.integral = 0;
    motor.last_error = 0;
    motor.speed(current_speed);
  }
  long startTime = millis();
  long startEnc = encoderTicks;
  delay(100);
  long endTime = millis();
  long endEnc = encoderTicks;
  float current = (float)(endEnc-startEnc)/(endTime-startTime) * 60000.0/960.0;
  //Serial.print(current);
  //Serial.print("    ");
  //Serial.println(current_speed);
  float error = current - rpm;
  float deriv = error-motor.last_error;
  motor.integral += error;
  float adjust = error*motor.kp + motor.integral * motor.ki + deriv*motor.kd;
  motor.current_speed -= adjust;
  motor.speed(current_speed);
}*/
