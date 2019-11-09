#include "main.h"
#include "pid.h"
#include "all_used.h"
#include "angler.h"
#include "motor_sensor_init.h"

float currentTarget;
float nextTarget;
float delayTime;
float nextDelayTime;
const float 7STACK_TORQUE = 1.45; // this is roughly the amount of torque on the motor for a 7 stack
bool anglerBool = false;
bool timerAng = false;
bool torqueCheck = false;

void angler_pid(float position, float delay) {
  // assigns position value based on if there is a currentTarget or not.
  if (!currentTarget) {
    currentTarget = position;
    delayTime = delay;
  } else {
    nextTarget = position;
    nextDelayTime = delay;
  }
  // runs tasks and its checks
  anglerBool = true;
  timerAng = true;
  torqueCheck = true;
}

void angler_pid_task(void*ignore) {
  pid_values angler_pid(0.5, 0.8, 0.8, 30, 500, 80);
  float holdTimer;
  float timeout;
  float maxTorque = 0;

  while(true) {
    while (anglerBool) {
      if (timerAng) {
        holdTimer = pros::millis() + delayTime; // motor hold value
        timeout = pros::millis() + 2000 + ((delayTime > 2000) ? 1000 : delayTime); // timeout value to exit out of the loop, if something goes wrong
        timerAng = false;
      }

      // max torque value is used to calculate how many cubes are in the angler
      if (pros::c::motor_get_torque(11) > maxTorque) {
        maxTorque = pros::c::motor_get_torque(11);
      }

      // slightly reduces the target of a 7 stack to improve accuracy
      if ((maxTorque > 7STACK_TORQUE) && torqueCheck) {
        currentTarget = currentTarget - 20;
        torqueCheck = false;
      }

      // run motors faster depending on the amount of torque applied on the motor
      // based on the number of cubes, the max power needs to be greater
      if (pros::c::motor_get_torque(11) > 0.7) {
        angler_pid.max_power = 100;
      } else {
        angler_pid.max_power = 80;
      }

      // slows down near the end of the stack
      if (fabs(angler_pid.error) < 600) {
        if (angler_pid.max_power < 10) {
          angler_pid.max_power = 10;
        } else {
          // slow down faster for 7 stack or greater
          if (maxTorque > 7STACK_TORQUE) {
            angler_pid.max_power = angler_pid.max_power - 25;
          } else {
            angler_pid.max_power = angler_pid.max_power - 15;
          }
        }
      }

      float currentTime = pros::millis();
      float position = potentiometer_angler.get_value();
      int final_power = pid_calc(&angler_pid, currentTarget, position);
      angler.move(final_power);

      // exits out of the loop after the +/- 10 of the error has been reached, hold value has been reached
      if (((fabs(angler_pid.error) <= 10) && (currentTime > holdTimer)) || (currentTime > timeout))  {
        angler.move(0);
        maxTorque = 0;
        // if there is a next target, then switch to the next target, else clear current target and exit the loop
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
      }

      pros::delay(20);
    }
    pros::delay(20);
  }
}
