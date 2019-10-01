#include "main.h"
#include "intake.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"


int speed;
int run_time;
pros::task_t loader_task;
void intake(void *ignore)
{
  while(true)
  {
  loader_right.move(speed);
  loader_left.move(-speed);
  pros::delay(run_time);
  loader_right.move(0);
  loader_left.move(0);
  }
}
