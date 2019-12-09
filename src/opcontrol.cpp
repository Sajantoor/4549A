#include "main.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "drive.h"
#include "all_used.h"
#include "lift.h"
#include "angler.h"

void opcontrol() {
	//full_position_reset();
	pros::ADIPort potentiometer_arm (pot_port_arm, pros::E_ADI_ANALOG_IN);
	pros::ADIPort potentiometer_angler (pot_port_angler, pros::E_ADI_ANALOG_IN);
	pros::Controller controller (pros::E_CONTROLLER_MASTER);
	bool liftVal = true;
	int stickArray[4];
	int power[4];

	while (true) {

		stickArray[0] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), 3) / powf(127, 2);
		stickArray[1] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 3) / powf(127, 2);
		stickArray[2] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 3) / powf(127, 2);
		stickArray[3] = powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), 3) / powf(127, 2);

		for (size_t j = 0; j < 4; j++) {
			if (abs(stickArray[j]) < 15) {
				stickArray[j] = 0;
			}

			if (j == 0 || j == 2) {
				if (127 - abs(stickArray[j]) < 30) {
					if (stickArray[j] > 0)
						stickArray[j] = 127;
					else
						stickArray[j] = -127;
				}
			}
		}

		power[0] = stickArray[1] + stickArray[0];
		power[1] = stickArray[1] - stickArray[0];
		power[2] = stickArray[3] - stickArray[2];
		power[3] = stickArray[3] + stickArray[2];

		for (size_t i = 0; i < 4; i++) {
			if (abs(power[i]) > 127) {
				if (power[i] > 0) {
					power[i] = 127;
				} else {
					power[i] = -127;
				}
			}
		}

		drive_left = power[0];
		drive_left_b = power[1];
		drive_right = power[2];
		drive_right_b = power[3];

		// loader
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			loader_left.move(127);
			loader_right.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			loader_left.move(-127);
			loader_right.move(-127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			loader_left.move(-43);
			loader_right.move(-43);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			loader_left.move(-43);
			loader_right.move(-43);
		} else {
			loader_left.move(0);
			loader_right.move(0);
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			if (liftVal) {
				angler_pid(1350, 4000);
				angler_pid(3550, 0);
			}
			// angler_pid(2570, 0);
			// pros::delay(1700);
			// intake_run(-127,2500);
			// angler_pid(1780, 0);
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			if (liftVal) {
				lift(1780, 20000);
			} else {
				lift(0, 0);
			}

			liftVal ? liftVal = false : liftVal = true;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			if (liftVal) {
				lift(2500, 20000);
			} else {
				lift(1100, 0);
			}

			liftVal ? liftVal = false : liftVal = true;
		}

		// if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
		// 	if (liftVal) {
		// 		angler_pid(2060, 20000);
		// 		lift(2570, 20000);
		// 	} else {
		// 		lift(3830, 0);
		// 		pros::delay(500);
		// 		angler_pid(1745, 0);
		// 	}
		//
		// 	liftVal ? liftVal = false : liftVal = true;
		// }


		// if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
		// 	if (liftVal) {
		// 		angler_pid(1569, 20000);
		// 		lift(1210, 20000);
		// 	} else {
		// 		lift(2600, 0);
		// 		pros::delay(500);
		// 		angler_pid(1745, 0);
		// 	}
		//
		// 	liftVal ? liftVal = false : liftVal = true;
		// }

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			arm.move(127);
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			arm.move(-127);
		} else {
			arm.move(0);
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			angler_pid(2000, 0);
		  angler_pid(1580, 0);
		  angler_pid(2000, 0);
		  loader_left.move(-127);
		  loader_right.move(-127);
		  pros::delay(1500);
		  loader_left.move(0);
		  loader_right.move(0);
		  angler_pid(1580, 0);

		}


		pros::delay(20);
	}
}
