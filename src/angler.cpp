#include "main.h"
#include "pid.h"
#include "all_used.h"
#include "angler.h"
#include "motor_sensor_init.h"

// global variables
const float SEVEN_STACK_TORQUE = 1; // this is roughly the amount of torque on the motor for a 7 stack
const float NINE_STACK_TORQUE = 4; // roughly the amount of torque for a 9 stack

float currentTarget;
float nextTarget;
float currentSpeed;
float nextSpeed;
float anglerDelay;

bool anglerBool = false;
bool timerAng = false;
bool anglerHold = false;
bool torqueCheck = false;
bool applyTorque = false;

void angler_pid(float position, bool holdVal, float speed, bool applyTorque, float delayTime) {
  // assigns position value based on if there is a currentTarget or not.
  if (!currentTarget) {
    currentTarget = position;
    currentSpeed = speed;
  } else {
    nextTarget = position;
    nextSpeed = speed;
  }
  // runs tasks and its checks
  anglerDelay = delayTime;
  anglerHold = holdVal;
  anglerBool = true;
  timerAng = true;
  torqueCheck = true;
  applyTorque = applyTorque;
}

void angler_pid_task(void*ignore) {
  pid_values angler_pid(0.5, 0.8, 0.8, 30, 500, 100);
  float timeout;
  float maxTorque = 0;
  bool delayReached = false;

  while(true) {
    while (anglerBool) {
      if (timerAng) {
        // holdTimer = pros::millis() + delayTime; // motor hold value
        timeout = pros::millis() + anglerDelay; // timeout value to exit out of the loop, if something goes wrong
        timerAng = false;
        !applyTorque ? torqueCheck = false : torqueCheck = true;
      }

      if (anglerDelay && (pros::millis() > timeout)) {
        delayReached = true;
      }

      // max torque value is used to calculate how many cubes are in the angler
      if (pros::c::motor_get_torque(ANGLER) > maxTorque) {
        maxTorque = pros::c::motor_get_torque(ANGLER);
      }

      // slightly increases the target of a 7 stack to improve accuracy
      if ((maxTorque > SEVEN_STACK_TORQUE) && torqueCheck) {
        currentTarget = currentTarget - 1000;
        torqueCheck = false;
      }

      // run motors faster depending on the amount of torque applied on the motor
      // based on the number of cubes, the max power needs to be greater
      // if (!pros::c::motor_get_torque(ANGLER) > 0.7) {
      //   angler_pid.max_power -= 20;
      // }

      // slows down near the end of the stack
      // if (fabs(angler_pid.error) < 600) {
      //   if (angler_pid.max_power < 50) {
      //     angler_pid.max_power = 50;
      //   } else {
      //     // 9 stack torque needs the intake to run or else the bottom cube isn't in place
      //     if (maxTorque > NINE_STACK_TORQUE) {
      //       angler_pid.max_power = angler_pid.max_power - 5;
      //       loader_left.move(90);
      //       loader_right.move(90);
      //     }
      //     // slow down faster for 7 stack or greater
      //     else if (maxTorque > SEVEN_STACK_TORQUE) {
      //       angler_pid.max_power = angler_pid.max_power - 5;
      //     } else {
      //       angler_pid.max_power = angler_pid.max_power - 5;
      //     }
      //   }
      // }

      float currentTime = pros::millis();
      float position = potentiometer_angler.get_value();
      int final_power = pid_calc(&angler_pid, currentTarget, position);
      angler.move(final_power);
      printf("angler pid: %f \n \n", angler_pid.error);
      // exits out of the loop after the +/- 10 of the error has been reached, hold value has been reached
      if ((fabs(angler_pid.error) <= 10) || !anglerHold || delayReached)  {
        angler.move(0);
        maxTorque = 0;
        // if there is a next target, then switch to the next target, else clear current target and exit the loop
        if (nextTarget == 0) {
          currentTarget = 0;
          currentSpeed = 0;
          anglerBool = false;
          delayReached = false;
        } else {
          currentTarget = nextTarget;
          currentSpeed = nextSpeed;
          nextSpeed = 0;
          nextTarget = 0;
          timerAng = true;
          delayReached = false;
        }
      }

      pros::delay(20);
    }
    pros::delay(20);
  }
}
