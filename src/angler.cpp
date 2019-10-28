#include "main.h"
#include "pid.h"
#include "all_used.h"
#include "angler.h"
#include "motor_sensor_init.h"

float target;
float delayTime;
bool anglerBool = false;
bool timerAng = false;


void angler_pid(float position, float delayTime) {
  target = position;
  delayTime = delayTime;
  anglerBool = true;
  timerAng = true;
}

void angler_pid_task(void*ignore) {
  pid_values angler_pid(0.4, 0.3, 0.15, 25, 500, 70);
  float failsafe;

  while(true) {
    while (anglerBool) {
      if (timerAng) {
        failsafe = pros::millis() + delayTime;
        timerAng = false;
      }

      float position = potentiometer_angler.get_value();
      int final_power = pid_calc(&angler_pid, target, position);
      angler.move(final_power);

      printf("error %f \n \n", angler_pid.error);

      if ((fabs(angler_pid.error) < 10) && (pros::millis() > failsafe))  {
        anglerBool = false;
      }
    }
    angler.move(0);
    pros::delay(20);
  }
}
