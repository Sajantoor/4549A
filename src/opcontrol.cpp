#include "main.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "drive.h"
#include "all_used.h"
#include "lift.h"
#include "angler.h"

bool run_shit = true;

void opcontrol() {
	//pros::lcd::initialize();
//arm.move_absolute(800,120);
//arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	full_position_reset();
	pros::ADIPort potentiometer_arm (pot_port_arm, pros::E_ADI_ANALOG_IN);
	pros::ADIPort potentiometer_angler (pot_port_angler, pros::E_ADI_ANALOG_IN);
	pros::Controller controller (pros::E_CONTROLLER_MASTER);

	while (true) {
			// printf("Back Encoder %d\n", back_encoder.get_value());
			// printf("Right Encoder: %d\n", right_encoder.get_value());
			// printf("Left Encoder %d\n", left_encoder.get_value());

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
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			int drive_left = (controller.get_analog(ANALOG_LEFT_Y) * 0.5);
			int drive_left_b = (controller.get_analog(ANALOG_LEFT_Y) * 0.5);
			int drive_right = (controller.get_analog(ANALOG_RIGHT_Y) * 0.5);
			int drive_right_b = (controller.get_analog(ANALOG_RIGHT_Y) * 0.5);
	  }

		// controller deadzone detection for both sticks
		if (abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) < 15) {
			 left_drive_set(0);
		} else {
			// slew rate calculation
			left_drive_set((powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 3)) / powf(127, 2));
		}

		if (abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) < 15) {
			right_drive_set(0);
		} else {
			// slew rate calculation
			right_drive_set((powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), 3)) / powf(127, 2));
		}

		// loader
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			loader_left.move(127);
			loader_right.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			loader_left.move(-127);
			loader_right.move(-127);
		} else {
			loader_left.move(0);
			loader_right.move(0);
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			angler_pid(1275);
			pros::delay(20);
			angler_pid(2200);
			printf("button pressed \n \n");
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			angler_pid(1720);
			lift(3400);
		}

		pros::delay(20);
	}
}
