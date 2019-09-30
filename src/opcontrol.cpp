#include "main.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "drive.h"
#include "all_used.h"
#include "lift.h"

void opcontrol() {
	//pros::lcd::initialize();

full_position_reset();

	pros::ADIPort potentiometer (pot_port, pros::E_ADI_ANALOG_IN);

	pros::Controller controller (pros::E_CONTROLLER_MASTER);

	while (true) {

		printf("back_encoder %d\n", back_encoder.get_value());
		float line_angle = nearestangle(0.4636,0);
		printf("nearest angle %f \n", line_angle);

		//pros::lcd::print(1, "encoder_left %d\n", left_encoder.get_value());
	  //pros::lcd::print(5, "velocity.a %f\n", velocity.a);

		//pros::lcd::print(3, "position,x %f\n", position.x);
	  //pros::lcd::print(4, "position.y %f\n", position.y);

		//pros::lcd::print(6, "velocity.x %f\n", velocity.x);
		//pros::lcd::print(7, "velocity.y %f\n", velocity.y);

		// //pros::lcd::print(6,"orientation %f\n", radToDeg(orientation));
		// //pros::lcd::print(7, "encoder_back %d\n", back_encoder.get_value());

		printf("radian value left %f\n", degrees_to_rad_left);
		printf("radian value left %f\n", degrees_to_rad_right);

		printf("velocity.x %f\n", velocity.x);
		printf("velocity.y %f\n", velocity.y);
		printf("velocity.a %f\n", velocity.a);



//AUTO SELECTOR


//DRIVE
		if(controller.get_digital (pros::E_CONTROLLER_DIGITAL_L1) == 1 && controller.get_digital (pros::E_CONTROLLER_DIGITAL_L2) == 1)
	{
		int drive_left = (controller.get_analog(ANALOG_LEFT_Y)*0.5);
		int drive_left_b = (controller.get_analog(ANALOG_LEFT_Y)*0.5);
		int drive_right = (controller.get_analog(ANALOG_RIGHT_Y)*0.5);
		int drive_right_b = (controller.get_analog(ANALOG_RIGHT_Y)*0.5);
  }
	else
	{
	    if (abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) < 15)
			{
			   left_drive_set(0);
      }


			else
			{
        left_drive_set((powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),3))/ powf(127,2));
      }

      if (abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) < 15)
			{
      right_drive_set(0);
      }

			else
			{
        right_drive_set((powf(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y),3))/ powf(127,2));
      }
  }

	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) == 1)
	{
		lift_target = 200;
	}

			pros::delay(20);
	}
}
