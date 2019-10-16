#include "main.h"
#include "pid.h"
#include "all_used.h"
#include "angler.h"
#include "motor_sensor_init.h"
#include <vector>

float target;
bool anglerBool = false;
// define array to prepare for multiple calls, reserverd 5 element slots, can be increased if needed.
// Needs to be reservered because memory issues.
std::vector<float> targetArray(5);

void angler_pid(float target) {
  anglerBool = true;
  // add element to last position of the array
  targetArray.push_back(target);
}

void angler_pid_task(void*ignore) {
  // struct with pid values
  pid_values angler_pid(0.31, 0.07, 0, 30, 500, 70);

  while(true) {
    // task needs to run forever to work, we have a bool when we need it to run
    while (anglerBool) {
      float position = potentiometer_angler.get_value();
      // target is equal to the first value in the array
      int final_power = pid_calc(&angler_pid, targetArray[0], position);
      angler.move(final_power);
      // error is small, delete current target value, this will cause the array to shift with the next position value
      if ((fabs(angler_pid.error) < 12)) {
        targetArray.erase(targetArray.begin());
        // BUG: might need to add a delay here if it switches too fast.
      } else if ((fabs(angler_pid.error) < 12) && (targetArray.size() == 0)) {
        // exit out of the loop, if no more pending position values and error is small
        targetArray.clear();
        anglerBool = false;
      }
    }
    angler.move(0);
    pros::delay(20);
  }
}
