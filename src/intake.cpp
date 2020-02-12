#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "intake.h"
#include "pid.h"

// outtaking using the light sensor
void sensor_outtake() {
  std::uint32_t now = pros::millis();
  if (light_sensor_intake.get_value() > 1850) {
    // old solution if needed
    // loader_left.move(-50);
    // loader_right.move(-50);
    // // pros::delay(1000);
    // pros::Task::delay_until(&now, 500);
    // loader_left.move(0);
    // loader_right.move(0);

    intakePIDFunc(-500, 127);
	}
}
// gloabsl for intake pid
bool intakeTaskBool = false;
float intakeTaskPosition;
float intakeTaskSpeed;
// function setting gloabs for the intake pid
void intakePIDFunc(float target, float speed) {
  intakeTaskPosition = target;
  intakeTaskSpeed = speed;
  loader_left.tare_position();
  loader_right.tare_position();
  intakeTaskBool = true;
}

// intake motor move relative pid
// ENHANCE: Didn't have time to tune PID but make PID loop (if need be)
void intakePID(void*ignore) {
  while(true) {
    while (intakeTaskBool) {
      float timeout = 2000 + pros::millis();
      float currentTime = pros::millis();
      printf("timeout %f \n \n ", timeout);

    	loader_left.move_relative(intakeTaskPosition, intakeTaskSpeed);
    	loader_right.move_relative(intakeTaskPosition, intakeTaskSpeed);

    	while (!((loader_left.get_position() < (intakeTaskPosition + 50)) && (loader_left.get_position() > (intakeTaskPosition - 50))) || (currentTime > timeout)) {
        currentTime = pros::millis();
        pros::delay(2);
    	}

      intakeTaskBool = false;
      pros::delay(20);
    }

    pros::delay(20);
  }
}

// globals for auto intake task
float autoIntakeSpeed;
bool autoIntakeBool;
// change global values
void autoIntakeFunc(float speed) {
  autoIntakeSpeed = speed;

  if (autoIntakeSpeed) {
    autoIntakeBool = true;
  }
}
// auto intake task
void autoIntake(void*ignore) {
  // time for intake to slow if intake doesn't detect cubes
  float lightSensorTimeout = 500;
  float timer;
  bool intakeTimeout = false;

  while (true) {
    while (autoIntakeBool) { // task running bool
      if (light_sensor_intake.get_value() < 1850) { // if cube is detected
        loader_left.move(autoIntakeSpeed);
        loader_right.move(autoIntakeSpeed);
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
