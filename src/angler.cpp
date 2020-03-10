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
const int SENSOR_OUTTAKE_THRESHOLD = 200;
// globals for pid loop
float target;
float speed;
float anglerTimeout;
float waitTilSettle;
bool anglerBool = false;
bool timerAng = false;
bool anglerIsHold = false;
bool anglerIntakeThreshold = true;

void angler_pid(float position, float speed, float timeoutTime, float waitTilSettle, bool isHold) {
  // set values for pid
  target = position;
  speed = speed;
  anglerTimeout = timeoutTime;
  anglerIsHold = isHold;
  waitTilSettle = waitTilSettle;
  // run pid
  timerAng = true;
  anglerBool = true;
}

void angler_pid_task(void*ignore) {
  pid_values angler_pid(0.5, 0.8, 0.8, 30, 500, 127);
  float maxTorque = 0;
  float intakeThresholdTimer;
  const int ERROR_THRESHOLD = 10;

  while(true) {
    while (anglerBool) {
      if (timerAng) {
        float timeThresholds = pros::millis() + SENSOR_OUTTAKE_THRESHOLD;
        anglerTimeout += timeThresholds; // timeout value to exit out of the loop, if something goes wrong
        waitTilSettle += timeThresholds;
        intakeThresholdTimer = timeThresholds;
        timerAng = false;
      }
      // angler stack code
      if (anglerIntakeThreshold) {
        // max torque value is used to calculate how many cubes are in the angler
        if (pros::c::motor_get_torque(ANGLER) > maxTorque) {
          maxTorque = pros::c::motor_get_torque(ANGLER);
        }

        if ((maxTorque > TEN_STACK_TORQUE) && (fabs(angler_pid.error) < 1700)) {
          if (angler_pid.max_power <= speed * 0.4) angler_pid.max_power = speed * 0.4;
          else angler_pid.max_power -= 25;
        }
        // 8 stack torque is faster than 7 stack
        else if ((maxTorque > EIGHT_STACK_TORQUE) && (fabs(angler_pid.error) < 1700)) {
          if (angler_pid.max_power <= speed * 0.55) angler_pid.max_power = speed * 0.55;
          else angler_pid.max_power -= 15;
        // 7 stack torque is slower
        }

        else if ((maxTorque > SEVEN_STACK_TORQUE) && (fabs(angler_pid.error) < 1500)) {
          if (angler_pid.max_power <= speed * 0.55) angler_pid.max_power = speed * 0.55;
          else angler_pid.max_power -= 25;
        }

        else {  // slow down for all cubes
          if (fabs(angler_pid.error) < 1300) {
            if (angler_pid.max_power <= speed * 0.55) angler_pid.max_power = speed * 0.55;
            else angler_pid.max_power -= 15;
          } else {
            angler_pid.max_power = speed;
          }
        }

        float currentTime = pros::millis();
        float position = angler.get_position();
        int final_power = pid_calc(&angler_pid, target, position);
        angler.move(final_power);
        // exits out of the loop after the +/- 10 of the error has been reached, hold value has been reached
        if ((fabs(angler_pid.error) <= ERROR_THRESHOLD) && (currentTime > waitTilSettle) || !anglerIsHold || (currentTime > anglerTimeout)) {
          angler.move(0);
          maxTorque = 0;
          anglerIntakeThreshold = false;
          anglerBool = false;
        }
      } else {
        // outtake cube to be at the bottom of the tray
        if (light_sensor_intake.get_value() < 1850) { // if cube is detected
          intakes(0);
          anglerIntakeThreshold = true;
        } else if (light_sensor_intake.get_value() > 1850) { // if cube isn't detected
          if (pros::millis() > intakeThresholdTimer) {
            intakes(0);
            anglerIntakeThreshold = true;
          } else {
            intakes(-70);
          }
        }

      }

      pros::delay(20);
    }
    pros::delay(20);
  }
}
