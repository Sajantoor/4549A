#include "main.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "drive.h"
#include "all_used.h"
#include "lift.h"
#include "angler.h"

float calcRoot(float val) {
	if (val < 0)
		return -sqrt(-rootCheck);
	else
		return sqrt(rootCheck);
}

float mecanumCalc(float x, float y) {
	float rootCheck;

	if (x > 0 && y > 0) rootCheck = powf(y, 2) + powf(x, 2);
	if (x < 0 && y < 0) rootCheck = -powf(y, 2) + -powf(x, 2);
	if (x < 0) rootCheck = powf(y, 2) + -powf(x, 2);
	if (y < 0) rootCheck = powf(y, 2) + powf(x, 2);

	return calcRoot(rootCheck);
}

void opcontrol() {
	//full_position_reset();
	pros::ADIPort potentiometer_arm (pot_port_arm, pros::E_ADI_ANALOG_IN);
	pros::ADIPort potentiometer_angler (pot_port_angler, pros::E_ADI_ANALOG_IN);
	pros::Controller controller (pros::E_CONTROLLER_MASTER);
	bool liftVal = true;

	while (true) {
		int stickArray[4];
		stickArray[0] = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		stickArray[1] = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		stickArray[2] = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		stickArray[3] = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		for (size_t j = 0; j < 4; j++) {
			if (abs(stickArray[j]) < 15) {
				stickArray[j] = 0;
			}
		}

		int power[4];

		power[0] = mecanumCalc(stickArray[0], stickArray[1]);
		power[1] = mecanumCalc(-stickArray[0], stickArray[1]);
		power[2] = mecanumCalc(-stickArray[2], stickArray[3]);
		power[3] = mecanumCalc(stickArray[2], stickArray[3]);

		// power[0] = stickArray[1] + stickArray[0];
		// power[1] = stickArray[1] - stickArray[0];
		// power[2] = stickArray[3] - stickArray[2];
		// power[3] = stickArray[3] + stickArray[2];

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
				angler_pid(2450, 2000);
				angler_pid(1580, 0);
			}
			// angler_pid(2570, 0);
			// pros::delay(1700);
			// intake_run(-127,2500);
			// angler_pid(1780, 0);
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			if (liftVal) {
				angler_pid(1950, 20000);
				lift(2820, 20000);
			} else {
				lift(1450, 0);
				pros::delay(500);
				angler_pid(1640, 0);
			}

			liftVal ? liftVal = false : liftVal = true;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			if (liftVal) {
				angler_pid(1820, 20000);
				lift(2500, 20000);
			} else {
				lift(1450, 0);
				pros::delay(500);
				angler_pid(1570, 0);
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
