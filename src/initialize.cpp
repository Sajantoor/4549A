#include "main.h"
#include "motor_sensor_init.h"
#include "drive.h"
#include "all_used.h"
#include "lcd.h"
#include "lift.h"
#include "angler.h"
#include "vision.h"


void initialize()  {
	// task init
	pros::task_t tracking_task = pros::c::task_create(tracking_update, (void*)NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "TRACKING TASK");
	pros::task_t velocity_task = pros::c::task_create(tracking_velocity, (void*)NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "VELOCITY TASK");
	pros::task_t auto_selecter_task = pros::c::task_create(auto_selecter, (void*)NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "AUTO SELECTER TASK");
	pros::task_t lift_task_init = pros::c::task_create(lift_task, (void*)NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "LIFT TASK");
	pros::task_t angler_task = pros::c::task_create(angler_pid_task, (void*)NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "ANGLER TASK");
	pros::task_t vision_task = pros::c::task_create(vision_tracking, (void*)NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "VISION TRACKING TASK");

}

void disabled() {}

void competition_initialize() {}
