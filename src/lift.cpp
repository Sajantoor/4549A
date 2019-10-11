#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "lift.h"
#include "pid.h"

#define PENCHOD 2000
//global variables

int height;

void lift(int value) {
  height = value;
}

void lift_task(void*ignore) {
  pid_values lift_pid(0.2, 0, 0, 30, 100, 110);

  while (true) {
    while (potentiometer_angler.get_value() > PENCHOD) {
       float position = potentiometer_arm.get_value();
       float final_power = pid_calc(&lift_pid, height, position);
       arm.move(final_power);
       pros::delay(5);
     }
  //  }
    pros::delay(20);
  }
}


// while(anglerPot > x){
//   run lift
//   pros::delay(20);
// }
