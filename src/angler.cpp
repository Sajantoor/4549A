#include "main.h"
#include "pid.h"
#include "all_used.h"
#include "angler.h"
#include "motor_sensor_init.h"
#include <vector>

// define array to prepare for multiple calls, reserverd 5 element slots, can be increased if needed.
// Needs to be reservered because memory issues.
std::vector<float> targetArray(10);
std::vector<float> delayArray(10);
float delayTime = 0;
bool anglerBool = false;
bool timerAng = false;
int queue = 0;

void angler_pid(float position, float delay) {
  targetArray[queue] = position;
  delayArray[queue] = delay;
  printf("target is: %f \n \n", targetArray[0]);
  anglerBool = true;
  timerAng = true;
  if (queue > 5) queue = 5; else queue++;
}

void angler_pid_task(void*ignore) {
  pid_values angler_pid(0.35, 0.8, 0.5, 30, 500, 80);
  float failsafe;

  while(true) {
    while (anglerBool) {
      if (timerAng) {
        failsafe = pros::millis() + delayArray[0];
        timerAng = false;
      }

      float currentTime = pros::millis();
      float position = potentiometer_angler.get_value();
      int final_power = pid_calc(&angler_pid, targetArray[0], position);
      angler.move(final_power);

      if (pros::c::motor_get_torque(11) > 0.7) {
        angler_pid.max_power = 100;
      } else {
        angler_pid.max_power = 80;
      }

      if (fabs(angler_pid.error) < 600) {
        if (angler_pid.max_power < 10) {
          angler_pid.max_power = 10;
        } else {
          angler_pid.max_power = angler_pid.max_power - 15;
        }
      }

      if (targetArray[0] == 0) {
        for (int j = 0; j < 5; j++) {
          targetArray[j] = 0;
          delayArray[j] = 0;
        }

        angler.move(0);
        queue = 0;
        printf("CLEARED ARRAY___________________________ \n \n");
        anglerBool = false;
      }

      if ((fabs(angler_pid.error) < 10) && (currentTime > failsafe) && !(targetArray.size() == 0))  {
        targetArray.erase(targetArray.begin());
        delayArray.erase(delayArray.begin());
        printf("SHIFTED ARRAY TO: %f ______________________________ \n \n \n", targetArray[0]);
        angler.move(0);
        timerAng = true;
        pros::delay(500);
      }
      pros::delay(20);
    }
    pros::delay(5);
  }
}
