#include "main.h"
#include "pid.h"
#include "all_used.h"
#include "angler.h"
#include "motor_sensor_init.h"

float target;

void angler_pid(float position) {
  target = position;
}

void angler_pid_task(void*ignore) {
  pid_values angler_pid(0.15, 0.07, 0, 30, 500, 70);

  while(true) {
    float position = potentiometer_angler.get_value();
    int final_power = pid_calc(&angler_pid, target, position);
    angler.move(final_power);
    pros::delay(20);
  }
}
