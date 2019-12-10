#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "lift.h"
#include "pid.h"

//global variables
int height;
bool liftBool = false;
bool timer = true;
// int angler_threshold = 2400;
int hold = 0;

void lift(int moveVal, int holdVal) {
  height = moveVal;
  hold = holdVal;
  liftBool = true;
  timer = true;
}

void lift_task(void*ignore) {
  int maxPower = 127;
  pid_values lift_pid(0.5, 0.7, 0, 30, 500, maxPower);
  float timeout = 1000;
  float failsafe;
  float delayTime;

  while (true) {
    // while ((potentiometer_angler.get_value() < angler_threshold) && liftBool) {
    while(liftBool) {
      if (timer) {
        failsafe = pros::millis() + timeout + hold;
        delayTime = pros::millis() + hold;
        timer = false;
      }

      if (pros::c::motor_get_torque(18) > 0.8) {
        lift_pid.max_power = 127;
      } else {
        lift_pid.max_power = maxPower;
      }

       float currentTime = pros::millis();
       float position = arm.get_position();
       float final_power = pid_calc(&lift_pid, height, position);
       arm.move(final_power);


       if ((fabs(lift_pid.error) < 10) && (currentTime > delayTime)) {
         liftBool = false;
         hold = 0;
         arm.move(0);
       } else if (failsafe < currentTime) {
         liftBool = false;
         hold = 0;
         arm.move(0);
       }

       printf("final power %f \n \n", final_power);
       printf("error %f \n \n", lift_pid.error);
     }

    pros::delay(20);
  }
}


// while(anglerPot > x){
//   run lift
//   pros::delay(20);
// }
