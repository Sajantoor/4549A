#include "main.h"
#include "motor_sensor_init.h"
#include "drive.h"
#include "all_used.h"
#include "lcd.h"
#include "lift.h"
#include "intake.h"
#include "angler.h"


void initialize()  {
		//pros::lcd::initialize();

	//pros::lcd::print(0, "initialized");


	pros::task_t tracking_task = pros::c::task_create(tracking_update, (void*)NULL, TASK_PRIORITY_DEFAULT,
															TASK_STACK_DEPTH_DEFAULT, "TRACKING TASK");

	pros::task_t velocity_task = pros::c::task_create(tracking_velocity, (void*)NULL, TASK_PRIORITY_DEFAULT,
															TASK_STACK_DEPTH_DEFAULT, "VELOCITY TASK");

	pros::task_t auto_selecter_task = pros::c::task_create(auto_selecter, (void*)NULL, TASK_PRIORITY_DEFAULT,
																		TASK_STACK_DEPTH_DEFAULT, "AUTO SELECTER TASK");

	pros::task_t lift_task_init = pros::c::task_create(lift_task, (void*)NULL, TASK_PRIORITY_DEFAULT,
														TASK_STACK_DEPTH_DEFAULT, "LIFT TASK");

	pros::task_t loader_task = pros::c::task_create(intake, (void*)NULL, TASK_PRIORITY_DEFAULT,
														TASK_STACK_DEPTH_DEFAULT, "LOADER TASK");

//	pros::task_t angler_task = pros::c::task_create(angler_pid_task, (void*)NULL, TASK_PRIORITY_DEFAULT,
											       //TASK_STACK_DEPTH_DEFAULT, "LOADER TASK");

pros::Task angler_task_cpp(angler_pid_task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,  "loader task");

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
