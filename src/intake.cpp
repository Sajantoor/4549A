#include "main.h"
#include "intake.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
void intake(int speed, int run_time)
{
  while(true)
  {
  loader_right.move (speed);
  loader_left.move (speed);
  pros::delay(run_time);
  loader_right.move (speed);
  loader_left.move (speed);
  }
}
