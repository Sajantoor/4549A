#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "lift.h"
#include "pid.h"

//global variabless
//pros::task_t lift_task;

//float lift_target;

void lift(float lift_target) {
  // while (true) {
    pid_values lift_pid(0.2, 0, 0, 30, 100, 110);
    int timeout = 100;
    int failsafe = 2000;    //2000
    int initial_millis = pros::millis();
    unsigned int net_timer;
    bool timer_turn = true;
    net_timer = pros::millis() + timeout;

    while ((pros::millis() < net_timer) && ((initial_millis + failsafe) > pros::millis())) {
      float encoder_avg = potentiometer_arm.get_value();
      float final_power = pid_calc(&lift_pid, lift_target, encoder_avg);
      printf("Final POWER %f\n", final_power);
      printf("what up %f\n", encoder_avg);

      arm.move(final_power);

      if (timer_turn == true) net_timer = pros::millis() + timeout;

      if (fabs(lift_pid.error) < 2) timer_turn = false;
      //47.5 SHORT POST
      //63 TALL POST
      pros::delay(20);
      lift_target = lift_pid.target;
    }
    arm.move(0);
  //}
}
