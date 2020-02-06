#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "intake.h"

float intakeSpeed;

void autoIntake(void*ignore) {
  while (true) {
    while (intakeSpeed && pros::competition::is_autonomous()) {
      if (light_sensor_intake.get_value() > 1850) {
        loader_left.move(intakeSpeed);
        loader_right.move(intakeSpeed);
      } else {
        loader_left.move(0);
        loader_right.move(0);
      }
      pros::delay(20);
    }

    pros::delay(20);
  }
}
