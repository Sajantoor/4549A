#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "lift.h"
#include "pid.h"

//global variables
int height;
bool liftBool = false;
bool timer = true;
const int ANGLER_THRESHOLD = 1720;
int hold = 0;

void lift(int moveVal, int holdVal) {
  height = moveVal;
  hold = holdVal;
  liftBool = true;
  timer = true;
}

void lift_task(void*ignore) {
  pid_values lift_pid(0.5, 0.7, 0, 30, 500, 127);
  float timeout = 1000;
  float failsafe;
  float delayTime;

  while (true) {
    while ((potentiometer_angler.get_value() < ANGLER_THRESHOLD) && liftBool) {
      if (timer) {
        failsafe = pros::millis() + timeout + hold;
        delayTime = pros::millis() + hold;
        timer = false;
      }

       float currentTime = pros::millis();
       float position = potentiometer_arm.get_value();
       float final_power = pid_calc(&lift_pid, height, position);


       if ((fabs(lift_pid.error) < 10) && (currentTime > delayTime)) {
         hold = 0;
         arm.move(0);
         liftBool = false;
       } else if (failsafe < currentTime) {
         hold = 0;
         arm.move(0);
         liftBool = false;
       }
       arm.move(final_power);

       printf("final power %f \n \n", final_power);
       pros::delay(20);
     }

    pros::delay(20);
  }
}


// while(anglerPot > x){
//   run lift
//   pros::delay(20);
// }
