#include "main.h"
#include "pid.h"
#include "all_used.h"
#include "angler.h"
#include "motor_sensor_init.h"

// global variables
const float SEVEN_STACK_TORQUE = 1.40;
const float EIGHT_STACK_TORQUE = 1.59;
const float NINE_STACK_TORQUE = 1.93;
const float TEN_STACK_TORQUE = 2.10;

const float TRAY_FORWARD_VAL = -4500;
const float TRAY_BACKWARD_VAL = 0;

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
bool anglerIntakeThreshold = true;
bool ignoreError = false;

void angler_pid(float position, bool holdVal, float speed, bool applyTorque, float delayTime, bool errorIgnore) {
  // assigns position value based on if there is a currentTarget or not.
  if (!currentTarget) {
    currentTarget = position;
    currentSpeed = speed;
    ignoreError = errorIgnore;
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
  anglerIntakeThreshold = false;
  applyTorque = applyTorque;
}

void angler_pid_task(void*ignore) {
  pid_values angler_pid(0.5, 0.8, 0.8, 30, 500, 127);
  float timeout;
  float maxTorque = 0;
  bool delayReached = false;
  float intakeThresholdTimer;
  const int ERROR_THRESHOLD = 10;

  while(true) {
    while (anglerBool) {
      if (timerAng) {
        // holdTimer = pros::millis() + delayTime; // motor hold value
        angler_pid.max_power = currentSpeed;
        timeout = pros::millis() + anglerDelay; // timeout value to exit out of the loop, if something goes wrong
        timerAng = false;

        !applyTorque ? torqueCheck = false : torqueCheck = true;
        intakeThresholdTimer = pros::millis() + 1000;
      }

      if (anglerIntakeThreshold || (currentTarget == TRAY_BACKWARD_VAL) || (currentTarget == -2200)) {
        // angler stack code
        anglerIntakeThreshold = true;
        if (anglerDelay && (pros::millis() > timeout)) {
          delayReached = true;
        }

        // max torque value is used to calculate how many cubes are in the angler
        if (pros::c::motor_get_torque(ANGLER) > maxTorque) {
          maxTorque = pros::c::motor_get_torque(ANGLER);
        }

        // 8 stack torque is faster than 7 stack
        if (maxTorque > EIGHT_STACK_TORQUE && (fabs(angler_pid.error) < 1700)) {
          if (angler_pid.max_power < currentSpeed * 0.55) {
            angler_pid.max_power = currentSpeed * 0.55;
          } else {
            angler_pid.max_power = angler_pid.max_power - 15;
          }
        // 7 stack torque is slower
        } else if (maxTorque > SEVEN_STACK_TORQUE && (fabs(angler_pid.error) < 1500)) {
          if (angler_pid.max_power < currentSpeed * 0.55) {
            angler_pid.max_power = currentSpeed * 0.55;
          } else {
            angler_pid.max_power = angler_pid.max_power - 25;
          }
        // slow down for all cubes
        } else {
          if (fabs(angler_pid.error) < 1300 && !(currentTarget == -2200)) {
            if (angler_pid.max_power < currentSpeed * 0.55) {
              angler_pid.max_power = currentSpeed * 0.55;
            } else {
              angler_pid.max_power = angler_pid.max_power - 15;
            }
          } else {
            angler_pid.max_power = currentSpeed;
          }
        }

        float currentTime = pros::millis();
        float position = angler.get_position();
        int final_power = pid_calc(&angler_pid, currentTarget, position);
        angler.move(final_power);
        // exits out of the loop after the +/- 10 of the error has been reached, hold value has been reached
        if ((fabs(angler_pid.error) <= ERROR_THRESHOLD) || !anglerHold || delayReached || (ignoreError && nextTarget)) {
          angler.move(0);
          maxTorque = 0;
          // if there is a next target, then switch to the next target, else clear current target and exit the loop
          if (nextTarget == 0) {
            currentTarget = 0;
            currentSpeed = 0;
            delayReached = false;
            anglerBool = false;
          } else {
            currentTarget = nextTarget;
            currentSpeed = nextSpeed;
            nextSpeed = 0;
            nextTarget = 0;
            timerAng = true;
            delayReached = false;
          }
        }
      } else {
        // outtake cube to be at the bottom of the tray
        if (light_sensor_intake.get_value() < 1850) { // if cube is detected
          loader_left.move(0);
          loader_right.move(0);
          anglerIntakeThreshold = true;
        } else if (light_sensor_intake.get_value() > 1850) { // if cube isn't detected
          // fixes bug if there is no cubes in tray and driver accidentally pressed stack button,
          // disabling the robot lol
          if (pros::millis() > intakeThresholdTimer) {
            loader_left.move(0);
            loader_right.move(0);
            anglerIntakeThreshold = true;
          } else {
            loader_left.move(-70);
            loader_right.move(-70);
          }
        }
      }

      pros::delay(20);
    }
    pros::delay(20);
  }
}
