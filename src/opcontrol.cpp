#include "main.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "drive.h"
#include "all_used.h"
#include "lift.h"
#include "angler.h"

void opcontrol() {
	// global variables
	//full_position_reset();
	pros::ADIPort potentiometer_arm (pot_port_arm, pros::E_ADI_ANALOG_IN);
	pros::ADIPort potentiometer_angler (pot_port_angler, pros::E_ADI_ANALOG_IN);
	pros::Controller controller (pros::E_CONTROLLER_MASTER);
	bool liftVal = true;
	bool anglerVal = true;
	int stickArray[4];
	int power[4];

	while (true) {
		printf("right encoder %d \n\n", right_encoder.get_value());
		printf("position.y %f \n\n", position.y);

		stickArray[0] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), 3) / powf(127, 2);
		stickArray[1] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 3) / powf(127, 2);
		stickArray[2] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 3) / powf(127, 2);
		stickArray[3] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), 3) / powf(127, 2);
		// loops through array and removes values under 15 from the calculation
		for (size_t j = 0; j < 4; j++) {
			if (abs(stickArray[j]) < 15) {
				stickArray[j] = 0;
			}
			// for x values remove if they are over 30
			if (j == 0 || j == 2) {
				if (127 - abs(stickArray[j]) < 30) {
					if (stickArray[j] > 0)
						stickArray[j] = 127;
					else
						stickArray[j] = -127;
				}
			}
		}
		// tank drive with mecanum calculations
		power[0] = stickArray[1] + stickArray[0];
		power[1] = stickArray[1] - stickArray[0];
		power[2] = stickArray[3] - stickArray[2];
		power[3] = stickArray[3] + stickArray[2];
		// loops through all power values to check if they are above the limit (127)
		for (size_t i = 0; i < 4; i++) {
			if (abs(power[i]) > 127) {
				if (power[i] > 0) {
					power[i] = 127;
				} else {
					power[i] = -127;
				}
			}
		}
		// sets drive to power
		drive_left = power[0];
		drive_left_b = power[1];
		drive_right = power[2];
		drive_right_b = power[3];

		// intake on triggers
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			loader_left.move(127);
			loader_right.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			loader_left.move(-127);
			loader_right.move(-127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			loader_left.move(-86);
			loader_right.move(-86);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			loader_left.move(-86);
			loader_right.move(-86);
		} else {
			loader_left.move(0);
			loader_right.move(0);
		}

		// autonomous stacking mechanism
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			if (liftVal && anglerVal) {
				angler_pid(1035, 20000);
			} else if (liftVal) {
				angler_pid(3665, 0, 80, false);
			}
			// same button to return
			anglerVal ? anglerVal = false : anglerVal = true;
		}
		// lift high scoring value
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			if (liftVal) {
				angler_pid(3100, 20000);
				lift(1780, 20000);
			} else {
				angler_pid(3665, 0, 80, false);
				lift(0, 0);
			}
			// same button to return
			liftVal ? liftVal = false : liftVal = true;
		}
		// lift low scoring value
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			if (liftVal) {
				angler_pid(3100, 20000);
				lift(2500, 20000);
			} else {
				angler_pid(3665, 0, 80, false);
				lift(900, 0);
			}
			// same button to return
			liftVal ? liftVal = false : liftVal = true;
		}
		// lift descore value
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			if (liftVal) {
				angler_pid(3100, 20000);
				lift(1600, 20000);
			} else {
				angler_pid(3665, 0, 80, false);
				lift(900, 0);
			}
			// same button to return
			liftVal ? liftVal = false : liftVal = true;
		}
		// unlocking mechanism
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
		  angler_pid(1580, 0);
			pros::delay(2000);
		  loader_left.move(-127);
		  loader_right.move(-127);
		  pros::delay(1500);
		  loader_left.move(0);
		  loader_right.move(0);
			angler_pid(3665, 0, 80, false);
		}

		pros::delay(20);
	}
}
