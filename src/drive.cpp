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

vector position;
float prevEncoderBack = 0;
float prevEncoderLeft = 0;
float prevEncoderRight = 0;
float orientation;
float beginning_orientation;

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

// --------------- TRACKING ------------------------------
void tracking_update(void*ignore) {
	while(true) {
		// encoder values coverted to radians, to inches
		float encoderLeft = degreesToRad(left_encoder.get_value()) * WHEEL_RADIUS;
		float encoderRight = degreesToRad(right_encoder.get_value()) * WHEEL_RADIUS;
		float encoderBack = degreesToRad(back_encoder.get_value()) * WHEEL_RADIUS;

		float changeInLeft = encoderLeft - prevEncoderLeft;
		float changeInRight = encoderRight - prevEncoderRight;
		float changeInBack = encoderBack - prevEncoderBack;

		// calculates the change in angle and updates the orientation value
		float changeInAngle = (changeInLeft - changeInRight) / (DISTANCE_BETWEEN_CENTER * 2);
		float newOrientation = changeInAngle + orientation;

		// change in x and y
		vector localOffset;

		if (changeInAngle == 0) {
			localOffset = { changeInBack, changeInRight };
		} else {
			// uses chord length value if change in angle
			// OPTIMIZE: Chord length formula could be a function
			localOffset = {
				2 * sin(changeInAngle / 2) * ((changeInBack / changeInAngle) + DISTANCE_BETWEEN_BACKWHEEL_CENTER), // x value
				2 * sin(changeInAngle / 2) * ((changeInRight / changeInAngle) + DISTANCE_BETWEEN_CENTER) // y value
			};
		}

		// orientation of the bot
		float averageOrientation = orientation + (changeInAngle / 2 );
		// convert from polar to get vector
		polar polarOffset = vector_to_polar(localOffset);
    polarOffset.theta += changeInAngle;
		vector globalOffset = polar_to_vector(polarOffset);
		//updates the x and y
		position.x += globalOffset.x;
		position.y += globalOffset.y;
		// reset for next cycle
		orientation = newOrientation;
		prevEncoderLeft = encoderLeft;
		prevEncoderRight = encoderRight;
		prevEncoderBack = encoderBack;
		pros::delay(10);
	}
}

void moveAndShit(float x, float y) {
	// pid to move the shit for x and y
	// mecanum shit with torque, strafe and throttle => 3 pid loops
	// target is x and y => point on field
	// x = strafe
	// y = ??

}
