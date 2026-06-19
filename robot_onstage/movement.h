//libraries
#include <Arduino.h>
#include <cmath>
#include <numbers>

//encoders
extern const uint8_t direction_pin;
extern const uint8_t encoder_pin;
extern volatile int64_t encoders;
void enc_update();

//motor init prototypes
int cmap(int);
void speed(int);

//motor object
struct Motor {
  uint8_t fpin;
  uint8_t rpin;
  void speed(int val);
};
extern Motor Motor1;
extern Motor Motor2;
extern Motor Motor3;

//movement functions
void motor(int speed1, int speed2, int speed3);
void vmotor(float Vx, float Vy, float* Vx_act = nullptr, float* Vy_act = nullptr);
