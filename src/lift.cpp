#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "lift.h"
#include "pid.h"

//global variables
int height;
bool liftBool = false;
bool timer = true;
int hold = 0;

// takes variables to the task
void lift(int moveVal, int holdVal) {
  height = moveVal;
  hold = holdVal;
  liftBool = true;
  timer = true;
}

// lift task which controls lift with PID and motor encoders
void lift_task(void*ignore) {
  pid_values lift_pid(0.5, 0.7, 0, 30, 500, 127);
  float timeout = hold;
  float failsafe;
  float delayTime;

  while (true) {
    while(liftBool) {
      // calculates time for timeout
      if (timer) {
        failsafe = pros::millis() + timeout + hold;
        delayTime = pros::millis() + hold;
        timer = false;
      }

       float currentTime = pros::millis();
       float position = arm.get_position(); // mtr encoders
       float final_power = pid_calc(&lift_pid, height, position); // final power is calculated using pid
       arm.move(final_power);
       // slew rate to slow down the motor based on the error value
       if (position > 1980) {
         lift_pid.max_power = lift_pid.max_power - 5; // slew rate
         if (lift_pid.max_power < 60) lift_pid.max_power = 60; // capping lowest possible speed
       } else {
         lift_pid.max_power = lift_pid.max_power + 25; // positive slew rate
         if (lift_pid.max_power > 127) lift_pid.max_power = 127; // motor cap
       }

       // exit out of the loop
       if ((fabs(lift_pid.error) < 10) && (currentTime > delayTime)) {
         lift_pid.max_power = 127;
         hold = 0;
         arm.move(0);
         liftBool = false;
       // exit out based off the failsafe
       } else if (failsafe < currentTime) {
         lift_pid.max_power = 127;
         hold = 0;
         arm.move(0);
         liftBool = false;
       }
     }
     pros::delay(20);
  }
}
