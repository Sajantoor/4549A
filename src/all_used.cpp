#include "main.h"
#include "all_used.h"
#include "drive.h"
#include "math.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "lift.h"

template <typename T> int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

// reset all position values
void full_position_reset() {
  drive_left.tare_position();
  drive_left_b.tare_position();
  drive_right.tare_position();
  drive_right_b.tare_position();
  left_encoder.reset();
  right_encoder.reset();
  back_encoder.reset();
  prev_inches_traveled_left = 0;
  prev_inches_traveled_right = 0;
  prev_inches_traveled_back = 0;
  beginning_orientation = 0;
  position.y = 0;
  position.x = 0;
  orientation = 0;
}
float nearestangle(float target_angle, float reference_angle) {
  return round((reference_angle-target_angle) / (2 * pi)) *  (2 * pi) + target_angle;
}

float flmod(float x, float y) {
	int q = floor(x / y);
	return x - (float)q * y;
}

float degToRad(float degrees) {
	return degrees * pi / 180;
}

float radToDeg(float radians) {
	return radians * 180 / pi;
}
void HarshStop()
{
	vector vel;
	vel.x = velocity.x;
	vel.y = velocity.y;
	polar polarVel;
	vectorToPolar(vel, polarVel);
	polarVel.theta += orientation;
	polarToVector(polarVel, vel);
	float yStop = vel.y, aStop = orientation, xStop = vel.x;

	printf("Vel y: %f | a: %f | x: %f \n \n", yStop, aStop, xStop);

	yStop *= -0.2;
	aStop *= -6.3;
  xStop *= -0.2;

  int left_b = yStop + aStop - xStop;
  int right_b = yStop - aStop + xStop;
	int left = yStop + aStop + xStop;
	int right = yStop - aStop - xStop;

  limit_to_val_set(left, 3);
  limit_to_val_set(right, 3);
  limit_to_val_set(left_b, 3);
  limit_to_val_set(right_b, 3);

	printf("Applying harsh stop: %d %d %d %d\n \n", left, right, left_b, right_b);
  drive_left.move(left);
  drive_left_b.move(left_b);
  drive_right.move(right);
  drive_right_b.move(right_b);
	pros::delay(150);
	set_drive(0, 0);
}
// reset position for tracking system
void reset_position_full(float x, float y, float a) {
	printf("Resetting position %f %f %f | %f %f %f \n \n", position.y, position.x, radToDeg(fmod(orientation, pi * 2)), y, x, radToDeg(fmod(a, pi * 2)));
  left_encoder.reset();
  right_encoder.reset();
  back_encoder.reset();
  prev_inches_traveled_left = 0;
  prev_inches_traveled_right = 0;
  prev_inches_traveled_back = 0;
  beginning_orientation = a;
	position.y = y;
	position.x = x;
	orientation = a;
}
