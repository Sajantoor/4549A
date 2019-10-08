#include "main.h"
#include "pid.h"
#include "all_used.h"
#include "angler.h"
#include "motor_sensor_init.h"


void angler_pid(float target) {
  pid_values angler_pid(0.45, 0.7, 0.25, 30, 50, 60);
  int timeout = 100;
  int failsafe = 2000;    //2000
  int initial_millis = pros::millis();
  unsigned int net_timer;
  bool timer_turn = true;
  net_timer = pros::millis() + timeout;

  while ((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis())) {
    float encoder_avg = potentiometer_angler.get_value();
    float final_power = pid_calc(&angler_pid, target, encoder_avg);

    angler.move(final_power);
    if (timer_turn == true) net_timer = pros::millis() + timeout;
    // if less than 2
    if (fabs(angler_pid.error) < 2) timer_turn = false;

    pros::delay(20); //20
  }

  angler.move(0);
}
