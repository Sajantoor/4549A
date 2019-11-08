#include "main.h"
#include "pid.h"
#include "all_used.h"
#include "angler.h"
#include "motor_sensor_init.h"

// define array to prepare for multiple calls, reserverd 5 element slots, can be increased if needed.
// Needs to be reservered because memory issues.
float currentTarget;
float nextTarget;
float delayTime;
float nextDelayTime;
bool anglerBool = false;
bool timerAng = false;
bool torqueCheck = true;

void angler_pid(float position, float delay) {
  anglerBool = true;
  timerAng = true;
  torqueCheck = true;

  if (!currentTarget) {
    currentTarget = position;
    delayTime = delay;
  } else {
    nextTarget = position;
    nextDelayTime = delay;
  }
}

void angler_pid_task(void*ignore) {
  pid_values angler_pid(0.5, 0.8, 0.8, 30, 500, 80);
  float delayTimer;
  float timeout;
  float maxTorque = 0;

  while(true) {
    while (anglerBool) {
      if (timerAng) {
        delayTimer = pros::millis() + delayTime;
        timeout = pros::millis() + 2000;
        timerAng = false;
      }

      if (pros::c::motor_get_torque(11) > maxTorque) {
        maxTorque = pros::c::motor_get_torque(11);
      }

      // if ((maxTorque > 1.45) && torqueCheck) {
      //   currentTarget = currentTarget - 20;
      //   torqueCheck = false;
      //         printf("max torque: %f \n \n", maxTorque);
      // }

      float currentTime = pros::millis();
      float position = potentiometer_angler.get_value();
      int final_power = pid_calc(&angler_pid, currentTarget, position);
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
          if (maxTorque > 1.45) {
            angler_pid.max_power = angler_pid.max_power - 25;
          } else {
            angler_pid.max_power = angler_pid.max_power - 15;
          }
        }
      }


      if ((fabs(angler_pid.error) <= 5) && (currentTime > delayTimer))  {
        angler.move(0);

        if (nextTarget == 0) {
          currentTarget = 0;
          delayTime = 0;
          anglerBool = false;
        } else {
          currentTarget = nextTarget;
          delayTime = nextDelayTime;
          nextTarget = 0;
          timerAng = true;
        }

      } else if ((currentTime > timeout)) {
        angler.move(0);
        timerAng = true;
        pros::delay(20);
      }

      pros::delay(20);
    }
    pros::delay(5);
  }
}
