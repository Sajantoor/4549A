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
int hold = 0;

void lift(int moveVal, int holdVal) {
  height = moveVal;
  liftBool = true;
  timer = true;
  hold = holdVal;
}

void lift_task(void*ignore) {
  pid_values lift_pid(0.65, 0, 0, 30, 500, 110);
  float failsafe;
  float delayTime;
  float timeout = 1000;

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

      // printf("error check %f\n \n", fabs(lift_pid.error));
    // printf("FINAL_POWER %f\n \n", final_power);
    // printf("Potentiometer value %i\n \n", potentiometer_angler.get_value());
    // printf((currentTime > delayTime) ? "true \n \n" : "false");
       if ((fabs(lift_pid.error) < 10) && (currentTime > delayTime)) {
         liftBool = false;
         hold = 0;
       } else if (failsafe < currentTime) {
         liftBool = false;
         hold = 0;
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
