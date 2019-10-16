#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "lift.h"
#include "pid.h"

//global variables
int height;
bool liftBool = false;
bool timer = true;
int angler_threshold = 1780;
bool hold = false;

void lift(int moveVal, bool holdVal) {
  height = moveVal;
  hold = holdVal;
  liftBool = true;
  timer = true;
}

void lift_task(void*ignore) {
  pid_values lift_pid(0.65, 0, 0, 30, 500, 110);
  float timeout = 1000;
  float failsafe;
  float delayTime;

  while (true) {
    while ((potentiometer_angler.get_value() < angler_threshold) && liftBool) {
      if (timer) {
        failsafe = pros::millis() + timeout;
        timer = false;
      }

       float currentTime = pros::millis();
       float position = potentiometer_arm.get_value();
       float final_power = pid_calc(&lift_pid, height, position);
       arm.move(final_power);

       if ((fabs(lift_pid.error) < 10) && !hold) {
         liftBool = false;
         hold = false;
       } else if ((failsafe < currentTime) && !hold) {
         liftBool = false;
         hold = false;
       }
     }
    arm.move(0);
    pros::delay(20);
  }
}


// while(anglerPot > x){
//   run lift
//   pros::delay(20);
// }
