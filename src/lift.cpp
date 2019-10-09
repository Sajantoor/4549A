#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "lift.h"
#include "pid.h"

//global variabless
pros::task_t lift_task;

int lift_target;

void lift(float target) {
  pid_values lift_pid(0.45, 0.7, 0.25, 30, 50, 110);
  int timeout = 100;
  int failsafe = 2000;    //2000
  int initial_millis = pros::millis();
  unsigned int net_timer;
  bool timer_turn = true;
  net_timer = pros::millis() + timeout;

  while (pros::competition::is_autonomous() && (pros::millis() < net_timer) && ((initial_millis + failsafe) > pros::millis())) {
    float encoder_avg = potentiometer_arm.get_value();
    float calc_power = pid_calc(&lift_pid, target, encoder_avg);
    float final_power = power_limit(lift_pid.max_power, calc_power);
    arm.move(final_power);

    if (timer_turn == true) net_timer = pros::millis() + timeout;

    if (fabs(lift_pid.error) < 2) timer_turn = false;

    pros::delay(20);
  }
  arm.move(0);
}
