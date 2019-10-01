#include "main.h"
#include "motor_sensor_init.h"
#include "drive.h"
#include "all_used.h"
#include "lcd.h"
#include "lift.h"
#include "intake.h"



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()  {
		//pros::lcd::initialize();

	//pros::lcd::print(0, "initialized");

	gyro.reset();

	pros::task_t tracking_task = pros::c::task_create(tracking_update, (void*)NULL, TASK_PRIORITY_DEFAULT,
															TASK_STACK_DEPTH_DEFAULT, "TRACKING TASK");

	pros::task_t velocity_task = pros::c::task_create(tracking_velocity, (void*)NULL, TASK_PRIORITY_DEFAULT,
															TASK_STACK_DEPTH_DEFAULT, "VELOCITY TASK");

	pros::task_t auto_selecter_task = pros::c::task_create(auto_selecter, (void*)NULL, TASK_PRIORITY_DEFAULT,
											TASK_STACK_DEPTH_DEFAULT, "AUTO SELECTER TASK");

	pros::task_t lift_task = pros::c::task_create(lift, (void*)NULL, TASK_PRIORITY_DEFAULT,
											TASK_STACK_DEPTH_DEFAULT, "LIFT TASK");

	pros::task_t loader_task = pros::c::task_create(intake, (void*)NULL, TASK_PRIORITY_DEFAULT,
											TASK_STACK_DEPTH_DEFAULT, "LOADER TASK");


	// if ((potentiometer.get_value()) < 400) {
	// 	//pros::lcd::print(3, "red back stack and park");
	// }
	//
	// else if ((500 < potentiometer.get_value()) && (potentiometer.get_value() < 900))  {
	// 	//pros::lcd::print(3, "red back cap");
	// }
	//
	//  else if ((1000 < potentiometer.get_value()) && (potentiometer.get_value() < 1400)) {
	// 	 //pros::lcd::print(3, "red front park");
	//  }
	//
	// else if ((1500 < potentiometer.get_value()) && (potentiometer.get_value() < 1800)) {
	// 	//pros::lcd::print(3, "blue front park");
	// }
	//
	// //blue
	// else if ((1900 < potentiometer.get_value()) && (potentiometer.get_value() < 2400)) {
	// 	//pros::lcd::print(3, "blue back cap");
	// }
	//
	// else if ((2500 < potentiometer.get_value()) && (potentiometer.get_value() < 3000))  {
	// 	//pros::lcd::print(3, "blue stack park");
	// }
	//
	// else if ((3100 < potentiometer.get_value()) && (potentiometer.get_value() < 3400)) {
	// 	//pros::lcd::print(3, "skills");
	// }
	//
	// else if ((3500 < potentiometer.get_value()) && (potentiometer.get_value() < 4095)) {
	// 	//pros::lcd::print(3, "testing");
	// }

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()  {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()  {}
