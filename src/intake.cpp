#include "main.h"
#include "motor_sensor_init.h"
#include "motor_setup.h"
#include "intake.h"
#include "pid.h"
#include "vision.h"

// basic intake function
void intakes(int speed) {
  intake_left.move(speed);
  intake_right.move(speed);
}

const int LIGHT_SENSOR_THRESHOLD = 1850;
// Globals for sensor outtake
int sensorTimer = 0;
bool sensorOutakeBool;
bool sensorTimeout;

// outtaking using the light sensor
void sensor_outtake() {
  sensorTimer = 0;
  sensorTimeout = false;
  sensorOutakeBool = true;
}
// outakes to the light sensor making sure the cube is in the intakes
void sensor_outtake_task(void*ignore) {
  const int SENSOR_TIMEOUT_THRESHOLD = 25;

  while (true) {
    while (sensorOutakeBool) {
      double sensorValue = light_sensor.get_value();
      sensorTimer++;
        // outtakes to the sensor value
      if (sensorValue > LIGHT_SENSOR_THRESHOLD && !sensorTimeout) {
        intakes(-127);
      } else {  // stop the intakes and exit loop
        intakes(0);
        sensorOutakeBool = false;
      }
      // timeout value
      if (sensorTimer > SENSOR_TIMEOUT_THRESHOLD && !sensorTimeout) {
        sensorTimeout = true;
      }

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
  autoIntakeSpeed ? autoIntakeBool = true : autoIntakeBool = false;
}
// auto intake task
void autoIntake(void*ignore) {
  // time for intake to slow if intake doesn't detect cubes
  float timer;
  bool intakeTimeout = false;

  while (true) {
    while (autoIntakeBool && pros::competition::is_autonomous()) {
      double lightSensorValue = light_sensor_intake.get_value();
      if ((lightSensorValue < LIGHT_SENSOR_THRESHOLD) || currentCube.size > CUBE_SIZE_THRESHOLD_MAX) { // if cube is detected
        intakes(127);
        timer = lightSensorTimeout + pros::millis(); // timer is updated
      } else if (intakeTimeout) { // stop intakes and exit loop
        intakes(0);
        intakeTimeout = false; // restart the timeout to have it run again if cube is detected
      } else if (timer < pros::millis()) { // run timer
        intakeTimeout = true;
      }

      pros::delay(20);
    }

    pros::delay(20);
  }
}
