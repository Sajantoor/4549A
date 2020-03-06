#include "main.h"
#include "motor_sensor_init.h"
#include "lift.h"
#include "angler.h"
#include "intake.h"

const int LIFT_HIGH = 2500;
const int LIFT_LOW = 1800;
const int DRIVE_POWER_LIMIT = 127;

void opcontrol() {
	int stickArray[4]; // temporary power until limited
	int power[4]; // power to the array
	bool intakeUsed = false; // used to check if intake is used in a task
	bool anglerVal = false; // manage angler's states on one button
	bool liftBool = false; // used to check first lift

	while (true) {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			stickArray[0] = DRIVE_POWER_LIMIT;
			stickArray[2] = DRIVE_POWER_LIMIT;
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			stickArray[0] = -DRIVE_POWER_LIMIT;
			stickArray[2] = -DRIVE_POWER_LIMIT;
		} else {
			stickArray[0] = 0;
			stickArray[2] = 0;
		}

		stickArray[1] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 3) / powf(DRIVE_POWER_LIMIT, 2);
		stickArray[3] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), 3) / powf(DRIVE_POWER_LIMIT, 2);

		// tank drive with mecanum calculations
		power[0] = stickArray[1] + stickArray[0];
		power[1] = stickArray[1] - stickArray[0];
		power[2] = stickArray[3] - stickArray[2];
		power[3] = stickArray[3] + stickArray[2];
		// loops through all power values to check if they are above the limit (127)
		for (size_t i = 0; i < 4; i++) {
			if (abs(power[i]) > DRIVE_POWER_LIMIT) {
				if (power[i] > 0) {
					power[i] = DRIVE_POWER_LIMIT;
				} else {
					power[i] = -DRIVE_POWER_LIMIT;
				}
			}
		}

		drive_left = power[0];
		drive_left_b = power[1];
		drive_right = power[2];
		drive_right_b = power[3];

		// check if intake is used in any task, letting driver use it.
		if (intakeTaskBool || !anglerIntakeThreshold || autoIntakeBool || sensorOutakeBool || driveBackToggle) {
			intakeUsed = true;
		} else {
			intakeUsed = false;
		}
		// intake on triggers
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !intakeUsed) {
			intake_left.move_voltage(12000);
			intake_right.move_voltage(12000);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !intakeUsed) {
			intake_left.move_velocity(-12000);
			intake_right.move_velocity(-12000);
		} else if (!intakeUsed) {
			intake_left.move(0);
			intake_right.move(0);
		}

		// autonomous stacking mechanism
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			// go forward
			if (!anglerVal) {
				anglerHold = false;
				angler_pid(-4500, true, 127, false);
			} else if (anglerVal) { // go backward
				anglerHold = false;
				angler_pid(0, true, 127, false, 2000);
			}
			// same button to return
			anglerVal = !anglerVal;
		}

		float armPosition = arm.get_position();

		// lift high scoring value
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			if (!(LIFT_HIGH + 100 > armPosition && armPosition > LIFT_HIGH - 100)) {
				lift(LIFT_HIGH, 20000);

				if (!liftBool) {
					liftBool = true;
					sensor_outtake();
				}
			}
		}

		// lift low value
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			if (!(LIFT_LOW + 100 > armPosition && armPosition > LIFT_LOW - 100)) {
				lift(LIFT_LOW, 20000);

				if (!liftBool) {
					liftBool = true;
					sensor_outtake();
				}
			}
		}

		// drop lift
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			lift(0, 1000);
			liftBool = false;
		}

		pros::delay(20);
	}
}
