#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "lift.h"
#include "pid.h"

//global variables
int height;
bool liftBool = false;
int angler_threshold = 1820;

void lift(int value) {
  height = value;
  liftBool = true;
}

void lift_task(void*ignore) {
  pid_values lift_pid(0.65, 0, 0, 30, 500, 110);

  while (true) {
    while ((potentiometer_angler.get_value() < angler_threshold) && liftBool) {
       float position = potentiometer_arm.get_value();
       float final_power = pid_calc(&lift_pid, height, position);
       arm.move(final_power);
       pros::delay(20);

    //   printf("error check %f\n \n", fabs(lift_pid.error));
    // printf("FINAL_POWER %f\n \n", final_power);
    // printf("Potentiometer value %i\n \n", potentiometer_angler.get_value());
       if (fabs(lift_pid.error) < 12.5) {
         liftBool = false;
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
