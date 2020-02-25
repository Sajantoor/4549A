#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "intake.h"
#include "pid.h"
#include "vision.h"

bool sensorOutakeBool;
int sensorTimer = 0;

void sensor_outtake_task(void*ignore) {
  bool sensorTimeout;

  while (true) {
    while (sensorOutakeBool) {
      double sensorValue = light_sensor_intake.get_value();
      sensorTimer++;
      printf("timer value %i: \n \n", sensorTimer);
      if (sensorValue > 1850 && !sensorTimeout) {
        loader_left.move(-127);
        loader_right.move(-127);
      } else  {
        loader_left.move(0);
        loader_right.move(0);
        sensorOutakeBool = false;
      }

      if (sensorTimer > 25) {
        sensorTimeout = true;
      } else {
        sensorTimeout = false;
      }

      pros::delay(20);
    }
    pros::delay(20);
  }
}
// outtaking using the light sensor
void sensor_outtake() {
  sensorTimer = 0;
  sensorOutakeBool = true;
}
// globals for intake pid
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
void intakePID(void*ignore) {
  while(true) {
    while (intakeTaskBool) {
      float timeout = 2000 + pros::millis();
      float currentTime = pros::millis();
      printf("entered function %f \n \n ", timeout);

    	loader_left.move_relative(intakeTaskPosition, intakeTaskSpeed);
    	loader_right.move_relative(intakeTaskPosition, intakeTaskSpeed);

    	while (!((loader_left.get_position() < (intakeTaskPosition + 50)) && (loader_left.get_position() > (intakeTaskPosition - 50))) || (currentTime > timeout)) {
        currentTime = pros::millis();
        pros::delay(2);
    	}

      printf("exited loop");
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
  } else {
    autoIntakeBool = false;
  }
}
// auto intake task
void autoIntake(void*ignore) {
  // time for intake to slow if intake doesn't detect cubes
  float lightSensorTimeout = 1500;
  float timer;
  bool intakeTimeout = false;

  while (true) {
    while (autoIntakeBool && pros::competition::is_autonomous()) { // task running bool
      double lightSensorValue = light_sensor_intake.get_value();
      if ((lightSensorValue < 1850) || currentCube.size > CUBE_SIZE_THRESHOLD_MAX) { // if cube is detected
        loader_left.move(127);
        loader_right.move(127);
        timer = lightSensorTimeout + pros::millis(); // timer is updated
      } else if (intakeTimeout) { // run slow intake speed
        loader_left.move(0);
        loader_right.move(0);
        intakeTimeout = false; // restart the timeout to have it run again if cube is detected
      } else if (timer < pros::millis()) { // run timer
        intakeTimeout = true;
      }

      pros::delay(20);
    }

    if (!pros::competition::is_autonomous()) {
      autoIntakeBool = false;
    }

    pros::delay(20);
  }
}
