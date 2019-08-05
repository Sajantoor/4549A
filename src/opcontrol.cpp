#include "main.h"
#include "motor_setup.h"
#include "stacker.h"
#include "motor_sensor_init.h"
#include "drive.h"
#include "all_used.h"


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::lcd::initialize();
	reset_drive_encoders();

	left_encoder.reset();
	right_encoder.reset();
	back_encoder.reset();


	pros::ADIPort potentiometer (pot_port, pros::E_ADI_ANALOG_IN);

	stacker.tare_position();

	pros::Controller controller (pros::E_CONTROLLER_MASTER);
position = {0,0};

	while (true) {
		//pros::lcd::print(6, "left_encoder %d\n", left_encoder.get_value());
    //pros::lcd::print(7, "right_encoder %d\n", right_encoder.get_value());
		//pros::lcd::print(6, "position.x %f\n", position.x);
printf("back_encoder %d\n", back_encoder.get_value());
		//AUTO SELECTOR
		//tracking_update(); //commented again by RITAM
		pros::lcd::print(1, "encoder_left %d\n", left_encoder.get_value());
	  pros::lcd::print(2, "encoder_right %d\n", right_encoder.get_value());

		pros::lcd::print(4, "position,x %f\n", position.x);
	  pros::lcd::print(5, "position.y %f\n", position.y);

		pros::lcd::print(6, "orientation %f\n", orientation);
		pros::lcd::print(7, "encoder_back %d\n", back_encoder.get_value());



		printf("radian value left %f\n", degrees_to_rad_left);
		printf("radian value left %f\n", degrees_to_rad_right);
		//pros::lcd::print(6, "radian value right %f\n", right_encoder.get_value());

		if((potentiometer.get_value()) < 400){
			pros::lcd::print(3, "red back stack and park");
		}

		else if((500 < potentiometer.get_value()) && (potentiometer.get_value() < 900)){
			pros::lcd::print(3, "red back cap");
		}

		 else if((1000 < potentiometer.get_value()) && (potentiometer.get_value() < 1400)){
			 pros::lcd::print(3, "red front park");
		 }

		else if((1500 < potentiometer.get_value()) && (potentiometer.get_value() < 1800)){
			pros::lcd::print(3, "blue front park");
		}

		//blue
		else if((1900 < potentiometer.get_value()) && (potentiometer.get_value() < 2400)){
			pros::lcd::print(3, "blue back cap");
		}

		else if((2500 < potentiometer.get_value()) && (potentiometer.get_value() < 3000)){
			pros::lcd::print(3, "blue stack park");
		}

		else if((3100 < potentiometer.get_value()) && (potentiometer.get_value() < 3400)){
			pros::lcd::print(3, "skills");
		}

		else if((3500 < potentiometer.get_value()) && (potentiometer.get_value() < 4095)){
			pros::lcd::print(3, "testing");
		}

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
          left_drive_set(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) );
        }
        if (abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) < 15)
        {
        right_drive_set(0);
        }
        else
        {
          right_drive_set(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) );
        }
      }
//DRIVE

//LOADER
			if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	    {
	      //loader.move(120);
				reset_drive_encoders();

				left_encoder.reset();
				right_encoder.reset();
				back_encoder.reset();
position = {0,0};
	    }
	    // else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
	    // {
	    // //loader.move(-120);
	    // }
	    // else
	    // {
	    // loader.move(0);
	    // }
//LOADER

//STACKER

if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) == 1)
{
	position_turn(90,450, 100);
}
else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
{
	position_turn(-90,450, 80);
}
else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
{
	stacker.move(-60);
	pros::delay(250);
	stacker.move(0);
	stacker.tare_position();
}
else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
{
stack(4800,100);
}
else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
{
	stack(500,50);
}
else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
{
stack(4500,100);
}

//STACKER


//ANGELER

if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
{
		puncher.tare_position();
	angler.move_absolute(950,200);
	if((angler.get_position() > 930 && (angler.get_position() < 980))){
	puncher.move_relative(-1820,100);
}
}
else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
{
	puncher.tare_position();
	angler.move_absolute(340,200);
if((angler.get_position()> 320 && (angler.get_position() < 400))){
	puncher.move_relative(-1820,100);
}
}
else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
{
angler.move(-30);
pros::delay(350);
angler.move(0);
angler.tare_position();
}

//ANGLER AND PUNCH

		pros::delay(20);
	}
}
