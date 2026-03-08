//libraries
#include <Arduino.h>
#include <cmath>

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

void motor(int, int, int);
void vmotor(float Vx, float Vy, float rotation);
