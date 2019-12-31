#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "math.h"
#include "all_used.h"
#include "lift.h"
#include "pid.h"

using namespace pros::literals;

// ---------------------- DATA -------------------------------------
//the radius of the tracking wheels
// distances are in inches
const float WHEEL_RADIUS = 1.3845055;
const float DISTANCE_BETWEEN_CENTER = 4.60091565;
const float DISTANCE_BETWEEN_BACKWHEEL_CENTER = 4.913425;

tracking trackingValues;

// ----------- CONVERSIONS -------------------------------------------
// converts vector to polar with input and return
polar vector_to_polar(vector v) {
  return { sqrt(powf(v.x, 2) + powf(v.y, 2)), atan2f(v.x, v.y) };
}

// converts polar to vector with input and return
vector polar_to_vector(polar p) {
  return { p.radius * sin(p.theta), p.radius * cos(p.theta)} ;
}

// calculates vector to polar, no return
void vectorToPolar(vector& vector, polar& polar) {
	if (vector.x || vector.y) {
		polar.radius = sqrt(vector.x * vector.x + vector.y * vector.y);
		polar.theta = atan2(vector.y, vector.x);
	} else polar.radius = polar.theta = 0;
}

// calculates polar to vector, no return
void polarToVector(polar& polar, vector& vector) {
	if (polar.radius) {
		vector.x = polar.radius * cos(polar.theta);
		vector.y = polar.radius * sin(polar.theta);
	} else vector.x = vector.y = 0;
}

float degreesToRad(float degrees) {
	return (degrees * (PI / 180));
}

float radToDegrees(float radians) {
	return (radians * (180/ PI));
}

float ticksToInches(float ticks) {
  return (ticks * WHEEL_RADIUS);
}

float inchesToTicks(float inches) {
  return (inches / WHEEL_RADIUS);
}

// --------------- TRACKING ------------------------------
void tracking_func(tracking * v) {
  // encoder values coverted to radians, to inches
  v-> encoderLeft = ticksToInches(degreesToRad(left_encoder.get_value()));
  v-> encoderRight = ticksToInches(degreesToRad(right_encoder.get_value()));
  v-> encoderBack = ticksToInches(degreesToRad(back_encoder.get_value()));

  v-> changeInLeft = v-> encoderLeft - v-> initialEncoderLeft;
  v-> changeInRight = v-> encoderRight - v-> initialEncoderRight;
  v-> changeInBack = v-> encoderBack - v-> initialEncoderBack;

  // calculates the change in angle and updates the orientation value
  v-> changeInAngle = (v-> changeInLeft - v-> changeInRight) / (DISTANCE_BETWEEN_CENTER * 2);
  v-> orientation = v-> changeInAngle + v-> initialOrientation;

  // change in x and y
  vector localOffset;

  if (v-> changeInAngle == 0) {
    localOffset = { v-> changeInBack, v-> changeInRight };
  } else {
    // uses chord length value if change in angle
    // OPTIMIZE: Chord length formula could be a function
    localOffset = {
      2 * sin(v-> changeInAngle / 2) * ((v-> changeInBack / v-> changeInAngle) + DISTANCE_BETWEEN_BACKWHEEL_CENTER), // x value
      2 * sin(v-> changeInAngle / 2) * ((v-> changeInRight / v-> changeInAngle) + DISTANCE_BETWEEN_CENTER) // y value
    };
  }

  // orientation of the bot
  v-> averageOrientation = v-> initialOrientation + (v-> changeInAngle / 2 );
  // convert from polar to get vector
  polar polarOffset = vector_to_polar(localOffset);
  polarOffset.theta += v-> changeInAngle;
  vector globalOffset = polar_to_vector(polarOffset);
  //updates the x and y
  v-> position.x += globalOffset.x;
  v-> position.y += globalOffset.y;
  // reset for next cycle
  v-> initialOrientation = v-> orientation;
  v-> initialEncoderLeft = v-> encoderLeft;
  v-> initialEncoderRight = v-> encoderRight;
  v-> initialEncoderBack = v-> encoderBack;
}

void tracking_update(void*ignore) {
	while(true) {
	   tracking_func(&trackingValues);
     pros::delay(10);
	}
}

void basicMovement(float x, float y, float angle) {
	// pid to move the shit for x and y
	// mecanum shit with torque, strafe and throttle => 3 pid loops
	// target is x and y => point on field
	// x = strafe
	// y = torque

  polar absoluteError = {25, 25};
  float timeout = pros::millis() + 8000;

  while (((absoluteError.radius > 0.22) || (fabs(absoluteError.theta) > 0.15)) && (timeout > pros::millis())) {
    pid_values xDirPID(25, 1, 0, 30, 500, 127);
    pid_values yDirPID(14.5, 0, 1, 30, 500, 127);
    pid_values rotationPID(150, 1, 10, 30, 500, 127);

    float xDir = pid_calc(&xDirPID, x, trackingValues.position.x);
    float yDir = pid_calc(&yDirPID, y, trackingValues.position.y);
    float rotation = pid_calc(&rotationPID, degreesToRad(angle), trackingValues.orientation);
    // error
    trackingValues.error.x = xDirPID.error;
    trackingValues.error.y = yDirPID.error;
    absoluteError = vector_to_polar(trackingValues.error);
    absoluteError.theta += rotationPID.error;

    float drive_left_power = xDir + yDir + rotation;
    float drive_left_b_power = -xDir + yDir + rotation;
    float drive_right_power = -xDir + yDir - rotation;
    float drive_right_b_power = xDir + yDir - rotation;

    drive_left.move(drive_left_power);
    drive_left_b.move(drive_left_b_power);
    drive_right.move(drive_right_power);
    drive_right_b.move(drive_right_b_power);

    printf("x: %f \n \n y: %f \n \n rotation: %f \n \n", trackingValues.error.x, trackingValues.error.y, absoluteError.theta);
    pros::delay(20);
  }
  printf("done driving \n \n");
  drive_left.move(0);
  drive_left_b.move(0);
  drive_right.move(0);
  drive_right_b.move(0);
}
