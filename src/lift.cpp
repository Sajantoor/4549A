#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "lift.h"
#include "pid.h"

//global variables
int height;
bool liftBool = false;
bool timer = true;
int angler_threshold = 1769;
int hold = 0;

void lift(int moveVal, int holdVal) {
  height = moveVal;
  hold = holdVal;
  liftBool = true;
  timer = true;
}

void lift_task(void*ignore) {
  pid_values lift_pid(0.25, 0, 0, 30, 500, 127);
  float timeout = 1000;
  float failsafe;
  float delayTime;

  while (true) {
    while ((potentiometer_angler.get_value() < angler_threshold) && liftBool) {
      if (timer) {
        failsafe = pros::millis() + timeout + hold;
        delayTime = pros::millis() + hold;
        timer = false;
      }

       float currentTime = pros::millis();
       float position = potentiometer_arm.get_value();
       float final_power = pid_calc(&lift_pid, height, position);
       arm.move(final_power);

       if ((fabs(lift_pid.error) < 50) && (currentTime > delayTime)) {
         liftBool = false;
         hold = 0;
         arm.move(0);
       } else if (failsafe < currentTime) {
         liftBool = false;
         hold = 0;
         arm.move(0);
       }
     }

    pros::delay(20);
  }
}


// while(anglerPot > x){
//   run lift
//   pros::delay(20);
// }
