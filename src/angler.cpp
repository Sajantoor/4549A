#include "main.h"
#include "pid.h"
#include "all_used.h"
#include "angler.h"
#include "motor_sensor_init.h"

float target;
bool anglerBool = false;

void angler_pid(float position) {
  target = position;
  anglerBool = true;
  printf(anglerBool ? "true" : "false");
}

void angler_pid_task(void*ignore) {
  pid_values angler_pid(0.31, 0.07, 0, 30, 500, 70);

  while(true) {
    while (anglerBool) {
      float position = potentiometer_angler.get_value();
      int final_power = pid_calc(&angler_pid, target, position);
      angler.move(final_power);
      printf("error check %f\n \n", fabs(angler_pid.error));

      if (fabs(angler_pid.error) < 12) {
        anglerBool = false;
      }
    }
    angler.move(0);
    pros::delay(20);
  }
}
