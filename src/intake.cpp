#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "intake.h"
#include "pid.h"

// outtaking using the light sensor
void sensor_outtake() {
  std::uint32_t now = pros::millis();
  if (light_sensor_intake.get_value() > 1850) {
  		// ENHANcE: For more consistency this should be written using PID loop instead of timed
  		loader_left.move(-50);
  		loader_right.move(-50);
  		pros::Task::delay_until(&now, 500); // delay until 500 millis after now
  		loader_left.move(0);
  		loader_right.move(0);
  	}
}

bool intakeTaskBool = false;
float intakeTaskPosition;
float intakeTaskSpeed;

void intakePIDFunc(float target, float speed) {
  intakeTaskPosition = target;
  intakeTaskSpeed = speed;
  loader_left.tare_position();
  loader_right.tare_position();
  intakeTaskBool = true;
}

void intakePID(void*ignore) {
  pid_values intake_pid(1, 0, 0, 30, 500, 127);

  while(true) {
    while (intakeTaskBool) {
      float encoderAverage = loader_left.get_position() + loader_right.get_position() / 2;
      // intake_pid.max_power = intakeTaskSpeed;
      int finalPower = pid_calc(&intake_pid, intakeTaskPosition, encoderAverage);
      loader_left.move(finalPower);
      loader_right.move(finalPower);
      printf("error %f \n \n", intake_pid.error);
      printf("power %f \n \n", finalPower);

      if (fabs(intake_pid.error) <= 10) {
        loader_left.move(0);
        loader_right.move(0);
        intakeTaskBool = false;
      }

      pros::delay(20);
    }

    pros::delay(20);
  }
}


float autoIntakeSpeed;
bool autoIntakeBool;

void autoIntakeFunc(float speed) {
  autoIntakeSpeed = speed;

  if (autoIntakeSpeed) {
    autoIntakeBool = true;
  }
}

void autoIntake(void*ignore) {
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
