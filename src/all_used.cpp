#include "main.h"
#include "all_used.h"
#include "drive.h"
#include "math.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "lift.h"


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

void reset_position_full(vector& gposition, float x, float y, float a)
{
	printf("Resetting position %f %f %f | %f %f %f", position.y, position.x, radToDeg(fmod(orientation, pi * 2)), y, x, radToDeg(fmod(a, pi * 2)));

  left_encoder.reset();
  right_encoder.reset();
  back_encoder.reset();
  prev_inches_traveled_left = 0;
  prev_inches_traveled_right = 0;
  prev_inches_traveled_back = 0;
  beginning_orientation = 0;
  
	position.y = y;
	position.x = x;
	orientation = a;
}
