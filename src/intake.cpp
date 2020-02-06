#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "intake.h"

float intakeSpeed;

void autoIntake(void*ignore) {
  float lightSensorTimeout = 500;
  float timer;
  bool intakeTimeout = false;

  while (true) {
    while (intakeSpeed && pros::competition::is_autonomous()) {
      printf("value %d \n \n", light_sensor_intake.get_value());
      if (light_sensor_intake.get_value() < 1850) {
        loader_left.move(intakeSpeed);
        loader_right.move(intakeSpeed);
        timer = lightSensorTimeout + pros::millis();
        // printf("intakingggggggg \n \n");
      } else if (intakeTimeout) {
        loader_left.move(80);
        loader_right.move(80);
        intakeTimeout = false;
        // printf("nooootttttt intakinggggg \n \n");
      } else if (timer < pros::millis()) {
        intakeTimeout = true;
      }

      pros::delay(20);
    }

    pros::delay(20);
  }
}
