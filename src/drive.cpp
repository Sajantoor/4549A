#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "math.h"
#include "all_used.h"
#include "lift.h"
#include "pid.h"

using namespace pros::literals;

// globals
float correction_turn;
float degrees_flag;
float prev_correction_turn;
float correction_drive;
float prev_correction_drive;
float drive_distance_correction;
float degrees_to_rad_left;
float degrees_to_rad_right;
float orientation;
float beginning_orientation;
float prev_inches_traveled_left;
float prev_inches_traveled_right;
float prev_inches_traveled_back;
vector position;
sVel velocity;

pros::task_t tracking_task;

template <typename T> int sgn(T val)
{
	return (T(0) < val) - (val < T(0));
}


//-------------------------------TRACKING VALUES------------------------------------------------------------------------------------------------------------------

polar vector_to_polar(vector v) {
	return{ sqrt(powf(v.x,2) + powf(v.y,2)), atan2f(v.x,v.y) };
}

vector polar_to_vector(polar p) {
	return{ p.r * sin(p.theta),p.r * cos(p.theta) };
}

void vectorToPolar(vector& vector, polar& polar) {
	if (vector.x || vector.y) {
		polar.r = sqrt(vector.x * vector.x + vector.y * vector.y);
		polar.theta = atan2(vector.y, vector.x);
	}
	else polar.r = polar.theta = 0;
}

void polarToVector(polar& polar, vector& vector) {
	if (polar.r) {
		vector.x = polar.r * cos(polar.theta);
		vector.y = polar.r * sin(polar.theta);
	}
	else vector.x = vector.y = 0;
}

void tracking_update(void*ignore) {
	while (true) {
		//gets the ticks from the each of encoders
		float degrees_encoder_left = (left_encoder.get_value());
		float degrees_encoder_right = (right_encoder.get_value());
		float degrees_encoder_back = (back_encoder.get_value());

		//converts each of the encoders ticks to degrees and returns value in radians
		float degrees_to_rad_left = (pi / 180) * degrees_encoder_left;
		float degrees_to_rad_right = (pi / 180) * degrees_encoder_right;
		float degrees_to_rad_back = (pi / 180) * degrees_encoder_back;

		//the radius of the tracking wheels
		const float wheel_radius = 1.3845055;

		//Finds how much each tracking wheel traveled in inches
		float inches_traveled_left = degrees_to_rad_left * wheel_radius; //gives back values in inches
		float inches_traveled_right = degrees_to_rad_right * wheel_radius; //gives back values in inches
		float inches_traveled_back = degrees_to_rad_back * wheel_radius; //gives back values in inches


		const float distance_between_centre = 4.60091565;//1.59437
		const float distance_between_backwheel_center = 4.913425;//2.5

		//Returns the orientation of the bot in radians
		float new_absolute_orientation = beginning_orientation + ((inches_traveled_left - inches_traveled_right) / (2 * distance_between_centre));

		//Returns how much it has rotated from its previous point in radians
		float change_in_angle = new_absolute_orientation - orientation;

		// The change in position from previous reset
		vector local_offset;

		//if it did not rotate then it calculates the offset normally
		if (change_in_angle == 0) {
			local_offset = { inches_traveled_back - prev_inches_traveled_back , inches_traveled_right - prev_inches_traveled_right };
		}
		//otherwise calculate the inches traveled according to how much it turned
		else {
			local_offset = { 2 * sin(change_in_angle / 2) * (((inches_traveled_back - prev_inches_traveled_back) / change_in_angle) + distance_between_backwheel_center),
			2 * sin(change_in_angle / 2) * ((inches_traveled_right - prev_inches_traveled_right) / change_in_angle + distance_between_centre) };
		}

		float average_orientation = orientation + (change_in_angle / 2);
		float rotation_amount = orientation + (change_in_angle) / 2;

		//converts local_offset to polar coordinates so that we can rotate by the rotation_amount
		polar offset_polar = vector_to_polar(local_offset);
		offset_polar.theta += rotation_amount;
		vector global_offset = polar_to_vector(offset_polar);
		//converts back to vector coordinates

		//updates the position.x and position.y
		position.x += global_offset.x;
		position.y += global_offset.y;

		//updates orienation values and the inches traveled by the tracking wheels
		orientation = new_absolute_orientation; //gives back value in radians
		prev_inches_traveled_left = inches_traveled_left;
		prev_inches_traveled_right = inches_traveled_right;
		prev_inches_traveled_back = inches_traveled_back;
		pros::delay(10);
	}
}

//tracks the current velocity of the bot
void tracking_velocity(void*ignore) {
	while (true) {
		unsigned long curTime = pros::millis();
		long passed = curTime - velocity.lstChecked;

		if (passed > 40) {
			float posA = orientation;
			float posY = position.y;
			float posX = position.x;
			velocity.a = ((posA - velocity.lstPosA) * 1000.0) / passed;
			velocity.y = ((posY - velocity.lstPosY) * 1000.0) / passed;
			velocity.x = ((posX - velocity.lstPosX) * 1000.0) / passed;
			velocity.lstPosA = posA;
			velocity.lstPosY = posY;
			velocity.lstPosX = posX;
			velocity.lstChecked = curTime;
		}

		pros::delay(10);
	}
}
//------------------------------------------------------------------------------------------------------------------

void drive_line_up(int speed, int run_time_drive) {
	drive_set(speed);
	pros::delay(run_time_drive);
	drive_set(0);
}
//-------------------------------------POSITION PIDS--------------------------------------------------------------------
void position_turn(float target, int timeout, int max_speed) {
	float kp = 125;//75.6
	float kd = 0;
	float ki = 0;
	float proportional, derivative, integral;

	float error;
	float final_power;
	float encoder_avg;
	int last_error = 0;
	int integral_limit = 50;

	//int max_speed = 100;
	bool timer_turn = true;
	unsigned int net_timer;

	int failsafe = 1500;
	int initial_millis = pros::millis();
	printf("orientation %f\n", orientation);
	printf("timer %i\n", pros::millis());


	while (pros::competition::is_autonomous() && (pros::millis() < net_timer) && ((initial_millis + failsafe) > pros::millis())) {
		//do a basic pid loop on orientation with the target as the ending angle
		encoder_avg = orientation;
		error = degToRad(target) - encoder_avg;
		derivative = (error - last_error)*kd;
		last_error = error;
		integral = error + integral;
		proportional = error * kp;

		printf("target %f\n", degToRad(target));
		printf("error %f\n", radToDeg(error));

		//integral limeter
		if (fabs(error) > (degToRad(22))) integral = 0;//22
		if (integral > integral_limit) integral = integral_limit;
		if (-integral < -integral_limit) integral = -integral_limit;

		final_power = proportional + derivative + (integral * ki);
		turn_set(final_power);

		if (timer_turn == true) {
			net_timer = pros::millis() + timeout;
		}

		if (fabs(error) < degToRad(1)) {
			timer_turn = false;
		}
		pros::delay(20);
	}

	HarshStop();

	printf("target %f\n", degToRad(target));
	printf("Degrees Turned from: %f to %f\n", error, orientation);
	printf("Degrees Turned from:%f to %f\n", radToDeg(error), radToDeg(orientation));
}

//turning to a angle in a different way
void position_turn2(float target_angle, tTurnDir turnDir, float ratio_full, int coast_power, float stop_offset_deg) {
	printf("Turning to %f\n", radToDeg(target_angle));

	// the actual orientation that the bot will end at, calculated based on ratio_full and the starting and ending orientations
	float endFull;

	// decides which way to turn depened on which ever direction is shortest and
	// fastest
	if (turnDir == ch) {
		if (fmod(target_angle - orientation, pi * 2) > pi) {
			turnDir = ccw;
		}
		else {
			turnDir = cw;
		}
	}

	// the bot keep going at full power until after the full ratio and then it
	// goes at coastPower until stopOffsetDeg
	switch (turnDir) {
	case cw:
		// calculates the target angle
		target_angle = orientation + flmod(target_angle - orientation, pi * 2);
		endFull = orientation * (1 - ratio_full) + target_angle * ratio_full;
		set_drive(-80, 80);

		while (orientation < endFull) {
			pros::delay(10);
		}

		set_drive(-coast_power, coast_power);

		while (orientation < target_angle - degToRad(stop_offset_deg)) {
			pros::delay(10);
		}

		set_drive(20, -20);
		pros::delay(100);
		set_drive(0, 0);

		break;

	// GOING COUNTER CLOCKWISE
  //For going counter clockwise, The signs were switched since this part of the code is for going counter clockwise
	case ccw:
		target_angle = orientation - flmod(orientation - target_angle, pi * 2);
		endFull = orientation * (1 - ratio_full) + target_angle * ratio_full;
		set_drive(80, -80);
		while (orientation > endFull) {
			pros::delay(10);
		}

		set_drive(coast_power, -coast_power);
		while (orientation > target_angle + degToRad(stop_offset_deg)) {
			pros::delay(10);
		}

		set_drive(-20, 20);
		pros::delay(150);
		set_drive(0, 0);

		break;

	default:
		break;
	}
}

//function to turn to a specific point
void position_face_point2(float target_x, float target_y, tTurnDir turnDir, float ratio_full, float coast_power, float offset, float stopOffsetDeg) {

	//the actual orientation that the bot will end at, calculated based on ratio_full and the starting and ending orientations
	float endFull, target;

	//decides which way to turn depened on which ever direction is shortest and fastest
	if (turnDir == ch) {
		if (fmod(atan2(target_x - position.x, target_y - position.y) + offset - orientation, pi * 2) > pi) {
			turnDir = ccw;
		}
		else {
			turnDir = cw;
		}
	}

	switch (turnDir) {
	case cw:
		//calculation of the target dependent on our current position
		target = orientation + flmod(atan2(target_x - position.x, target_y - position.y) + offset - orientation, pi * 2);
		endFull = orientation * (1 - ratio_full) + target * ratio_full;

		set_drive(-80, 80);
		while (orientation < endFull) {
			pros::delay(10);
		}

		set_drive(-coast_power, coast_power);

		while (orientation < nearestangle(atan2(target_x - position.x, target_y - position.y) + offset, target) - degToRad(stopOffsetDeg) /*&& (velSafety? NOT_SAFETY(power, turnToTargetNewAlg) : 1 )*/) {
			pros::delay(10);
		}

		set_drive(20, -20);
		pros::delay(150);
		set_drive(0, 0);

		break;

		//For going counter clockwise, The signs were switched since this part of the code is for going counter clockwise
	case ccw:
		target = orientation - fmod(orientation - atan2(target_x - position.x, target_y - position.y) - offset, pi * 2);
		endFull = orientation * (1 - ratio_full) + (target)* ratio_full;

		set_drive(80, -80);

		while (orientation > endFull) {
			pros::delay(10);
		}

		set_drive(coast_power, -coast_power);

		while (orientation > nearestangle(atan2(target_x - position.x, target_y - position.y) + offset, target) + degToRad(stopOffsetDeg)) {
			pros::delay(10);
		}

		set_drive(-20, 20);
		pros::delay(150);

		set_drive(0, 0);

		printf("done \n");
		break;

	default:
		break;
	}
}

//same principle as position_face_point2 but using pid loops
void position_face_point(float target_x, float target_y, int timeout) {
	vector error;
	float Kp = 77; //0.2

	float kd = 0.5;//70   3
	float ki = 0;//0.06   0
	float proportional, derivative, integral;
	float final_power;
	float encoder_avg;
	int last_error = 0;
	int integral_limit = 50;

	int max_speed = 110; //90
	float max_error = 0.001f;
	bool timer_turn = true;
	unsigned int net_timer;

	int failsafe = 2000;
	int initial_millis = pros::millis();

	float direction_face;

	float encoder_average;
	float error_p;

	while (pros::competition::is_autonomous() && (pros::millis() < net_timer) && ((initial_millis + failsafe) > pros::millis())) {
		error.x = target_x - position.x;
		error.y = target_y - position.y;

		//what direction to face (target)
		direction_face = atan2f(error.y, error.x);

		encoder_avg = orientation;
		error_p = direction_face - encoder_avg;
		derivative = (error_p - last_error)*kd;
		last_error = error_p;
		integral = (error_p + integral);
		proportional = error_p * Kp;

		// start integral when target is less than 22 degrees
		if (fabs(error_p) > degToRad(22)) { integral = 0; }

		if (integral > integral_limit) { integral = integral_limit; }

		if (-integral < -integral_limit) { integral = -integral_limit; }


		final_power = proportional + derivative + (integral * ki);

		//pros::lcd::print(6, "final_power %f\n", final_power);

		turn_set(final_power);

		printf("direction_face %f\n", direction_face);
		printf(" \n");
		printf("error_p %f\n", error_p);
		printf(" \n");

		if (timer_turn == true) {
			net_timer = pros::millis() + timeout;
		}

		if (fabs(error_p) < degToRad(1)) {
			timer_turn = false;
		}
		pros::delay(20);
	}

	HarshStop();
	printf("target %f\n", radToDeg(direction_face));
	printf("target radians %f\n", direction_face);
	printf("Degrees Turned from: %f to %f\n", error_p, orientation);
	printf("Degrees Turned from:%f to %f\n", radToDeg(error_p), radToDeg(orientation));
}

void position_drive2(float ending_point_x, float ending_point_y, float target_angle, float max_power, unsigned int timeout) {
	vector positionErr;
	vector rotated_motorPower;
	vector rotation_vector;
	vector delta_main_line;
	polar positionErrPolar;
	polar rotated_motorPowerPolar;

	pid_values turn_pid(300, 3, 6, 30, 500, 127);//78
	pid_values xDir_pid(28, 0, 0, 30, 500, 127);//17.5
	pid_values yDir_pid(12, 8, 0, 30, 500, 127);//11.7,5

	//timeout on the code so that if it ever gets stuck in the while loop it exits after a certain amount of time
	//int timeout = 9000;
	unsigned int net_timer;
	int initial_millis = pros::millis();
	net_timer = initial_millis + timeout; //just to initialize net_timer at first
	int last_y = 0;
	int last_x = 0;

	bool timer_drive = true;

	float powf_of_X_Y;
	float magnitude_of_X_Y;
	float drive_left_power;
	float drive_left_b_power;
	float drive_right_power;
	float drive_right_b_power;
	float largestVal;
	printf("Moving to %f %f \n", ending_point_x, ending_point_y);
	do {
		largestVal = 0;
		//runs pid loops on the position.x and position.y and orienation
		float final_power_turn = pid_calc(&turn_pid, degToRad(target_angle), orientation);
		float final_power_xDir = pid_calc(&xDir_pid, ending_point_x, position.x);
		float final_power_yDir = pid_calc(&yDir_pid, ending_point_y, position.y);

		//assigns the final_power_strafe and throttle to a vector
		rotated_motorPower.y = final_power_yDir;
		rotated_motorPower.x = final_power_xDir;

		//The vector then is rotated so the frame of reference is robot centric besides field centric
		vectorToPolar(rotated_motorPower, rotated_motorPowerPolar);
		rotated_motorPowerPolar.theta += orientation;
		polarToVector(rotated_motorPowerPolar, rotated_motorPower);

		//applying slew rate on the motors so they dont burn out and there arent sudden movements
		limit_to_val_set(rotated_motorPower.y, abs(max_power));
		int delta_y = rotated_motorPower.y - last_y;
		limit_to_val_set(delta_y, 3);
		rotated_motorPower.y = last_y += delta_y;

		limit_to_val_set(rotated_motorPower.x, abs(max_power));
		int delta_x = rotated_motorPower.x - last_x;
		limit_to_val_set(delta_x, 5);
		rotated_motorPower.x = last_x += delta_x;

		drive_left_power = rotated_motorPower.y + final_power_turn + rotated_motorPower.x;
		drive_left_b_power = rotated_motorPower.y + final_power_turn - rotated_motorPower.x;
		drive_right_power = rotated_motorPower.y - final_power_turn - rotated_motorPower.x;
		drive_right_b_power = rotated_motorPower.y - final_power_turn + rotated_motorPower.x;


		float motor_power_array[4] = { drive_left_power, drive_left_b_power, drive_right_power, drive_right_b_power };

		//assigns largestVal to the highest value of the motor powers
		for (size_t i = 0; i < 4; i++) {
			if (abs(motor_power_array[i]) > largestVal) {
				largestVal = abs(motor_power_array[i]);
			}
		}

		//Scales down all the motor_power if the largestVal is over 127, this is to make sure the motors arent getting power over 127
		if (largestVal > 127) {
			motor_power_array[0] = (motor_power_array[0] * 127) / abs(largestVal);
			motor_power_array[1] = (motor_power_array[1] * 127) / abs(largestVal);
			motor_power_array[2] = (motor_power_array[2] * 127) / abs(largestVal);
			motor_power_array[3] = (motor_power_array[3] * 127) / abs(largestVal);
		}

		//applies power to the motors in mecanum formation
		drive_left.move(motor_power_array[0]);
		drive_left_b.move(motor_power_array[1]);
		drive_right.move(motor_power_array[2]);
		drive_right_b.move(motor_power_array[3]);

		if(magnitude_of_X_Y < 5) {
			limit_to_val_set(rotated_motorPower.y, abs(max_power));
			if (abs(rotated_motorPower.y) < abs(max_power)) {
				if (rotated_motorPower.y > 0) {
					rotated_motorPower.y -= 15;
				} else {
					rotated_motorPower.y += 15;
				}
			}

			limit_to_val_set(rotated_motorPower.x, abs(max_power));
			if (abs(rotated_motorPower.x) < abs(max_power)) {
				if (rotated_motorPower.x > 0) {
					rotated_motorPower.x -= 15;
				} else {
					rotated_motorPower.x += 15;
				}
			}
		}

		//this gets the magnitude of the error using the error from throttle and strafe
		powf_of_X_Y = powf(yDir_pid.error, 2) + powf(xDir_pid.error, 2);
		magnitude_of_X_Y = sqrtf(powf_of_X_Y);

		pros::delay(10);

	} while ((magnitude_of_X_Y > 1 || abs(radToDeg(turn_pid.error)) > 0.5) && (pros::millis() < net_timer));

	//applies harsh stop depending on how fast the robot was moving
	HarshStop();
	printf("driving done\n");
}
