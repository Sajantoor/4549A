#include "main.h"
#include "pid.h"
#include "all_used.h"
#include "angler.h"
#include "motor_sensor_init.h"

float target;
float delayTime = 0;
bool anglerBool = false;
bool timerAng = false;


void angler_pid(float position, float delay) {
  target = position;
  delayTime = delay;
  anglerBool = true;
  timerAng = true;
}

void angler_pid_task(void*ignore) {
  pid_values angler_pid(0.35, 0.8, 0.5, 30, 500, 90);
  float failsafe;

  while(true) {
    while (anglerBool) {
      if (timerAng) {
        failsafe = pros::millis() + delayTime;
        timerAng = false;
      }

      float currentTime = pros::millis();
      float position = potentiometer_angler.get_value();
      int final_power = pid_calc(&angler_pid, target, position);
      angler.move(final_power);

      if (pros::c::motor_get_torque(11) > 0.7) {
        angler_pid.max_power = 110;
      } else {
        angler_pid.max_power = 90;
      }

      if (fabs(angler_pid.error) < 600) {
        if (angler_pid.max_power < 10) {
          angler_pid.max_power = 10;
        } else {
          angler_pid.max_power = angler_pid.max_power - 15;
        }
      }

      // printf("torque %f \n \n", pros::c::motor_get_torque(11));
      printf("power %i \n \n", final_power);
      // printf("error %f \n \n", angler_pid.error);


      if ((fabs(angler_pid.error) < 10) && (currentTime > failsafe))  {
        anglerBool = false;
        angler.move(0);
        delayTime = 0;
      }
    }

    pros::delay(20);
  }
}
