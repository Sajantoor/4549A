#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "intake.h"

float intakeSpeed;
bool intakeBool;

void autoIntakeFunc(float speed) {
  intakeSpeed = speed;

  if (intakeSpeed) {
    intakeBool = true;
  }
}


void autoIntake(void*ignore) {
  float lightSensorTimeout = 500;
  float timer;
  bool intakeTimeout = false;

  while (true) {
    while (intakeBool) { // task running bool
      if (light_sensor_intake.get_value() < 1850) { // if cube is detected
        loader_left.move(intakeSpeed);
        loader_right.move(intakeSpeed);
        timer = lightSensorTimeout + pros::millis(); // timer is updated
      } else if (intakeTimeout) { // run slow intake speed
        loader_left.move(80);
        loader_right.move(80);
        intakeTimeout = false; // restart the timeout to have it run again if cube is detected
      } else if (timer < pros::millis()) { // run timer
        intakeTimeout = true;
      }

      pros::delay(20);
    }

    pros::delay(20);
  }
}
