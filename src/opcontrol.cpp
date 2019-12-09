#include "main.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "drive.h"
#include "all_used.h"
#include "lift.h"
#include "angler.h"

void opcontrol() {
	//pros::lcd::initialize();
//arm.move_absolute(800,120);
//arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	//full_position_reset();
	pros::ADIPort potentiometer_arm (pot_port_arm, pros::E_ADI_ANALOG_IN);
	pros::ADIPort potentiometer_angler (pot_port_angler, pros::E_ADI_ANALOG_IN);
	pros::Controller controller (pros::E_CONTROLLER_MASTER);
	bool liftVal = true;

	while (true) {
		// printf("position.x %f \n\n", position.x);
		// printf("position.y %f \n\n", position.y);
		//
		// 	printf("Back Encoder %d\n\n", back_encoder.get_value());
		// 	printf("Right Encoder: %d\n\n", right_encoder.get_value());
		// 	printf("Left Encoder %d\n\n", left_encoder.get_value());
		// 	// printf("position.x %f\n", position.x);
		// 	// printf(" \n");
		// 	// printf("position.y %f\n", position.y);
		// 	// printf(" \n");
		// 	printf("orientation %f\n", radToDeg(orientation));
		// 	printf(" \n");

			// float line_angle = nearestangle(0.4636,0);
			// printf("nearest angle %f \n", line_angle);

			//pros::lcd::print(1, "encoder_left %d\n", left_encoder.get_value());
		  //pros::lcd::print(5, "velocity.a %f\n", velocity.a);

			//pros::lcd::print(3, "position,x %f\n", position.x);
		  //pros::lcd::print(4, "position.y %f\n", position.y);

			//pros::lcd::print(6, "velocity.x %f\n", velocity.x);
			//pros::lcd::print(7, "velocity.y %f\n", velocity.y);

			// //pros::lcd::print(6,"orientation %f\n", radToDeg(orientation));
			// //pros::lcd::print(7, "encoder_back %d\n", back_encoder.get_value());

			// printf("radian value left %f\n", degrees_to_rad_left);
			// printf("radian value left %f\n", degrees_to_rad_right);
			//
			// printf("velocity.x %f\n", velocity.x);
			// printf("velocity.y %f\n", velocity.y);
			// printf("velocity.a %f\n", velocity.a);

	//DRIVE

	// slow down when buttons are pressed, for precise control
		// if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		// 	int drive_left = (controller.get_analog(ANALOG_LEFT_Y) * 0.5);
		// 	int drive_left_b = (controller.get_analog(ANALOG_LEFT_Y) * 0.5);
		// 	int drive_right = (controller.get_analog(ANALOG_RIGHT_Y) * 0.5);
		// 	int drive_right_b = (controller.get_analog(ANALOG_RIGHT_Y) * 0.5);
	  // }
		//
		// // controller deadzone detection for both sticks
		//
		// if ((fabs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) + fabs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y))) > (fabs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) + fabs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)))) {
		// 	set_drive((powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 3)) / powf(127, 2), (powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), 3)) / powf(127, 2));
		// } else {
		// 	strafe(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
		// 	strafe(-controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		// }

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
		// int left_power;
		// int left_b_power;

		int power[4];
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

		// if (LStickX < 0) {
		// 	left_power = -sqrt(powf(LStickY, 2) - powf(LStickX, 2));
		// 	left_b_power = -sqrt(powf(LStickY, 2) + powf(LStickX, 2));
		// } else {
		// 	left_power = sqrt(powf(LStickY, 2) + powf(LStickX, 2));
		// 	left_b_power = sqrt(powf(LStickY, 2) - powf(LStickX, 2));
		// }
		//
		// if (LStickY < 0) {
		// 	left_power = -left_power;
		// 	left_b_power = -left_b_power;
		// }

		// printf("LStickX %i \n \n", LStickX);
		// printf("LStickY %i \n \n", LStickY);
		// printf("RStickX %i \n \n", RStickX);

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
